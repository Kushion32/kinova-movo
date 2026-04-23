#!/usr/bin/env python3
"""
Person Following Monitor for MOVO Robot Navigation

Uses leg_detector output (/leg_tracker_measurements)
to check if a person is following behind the robot during navigation.

Publishes:
  /movo/person_following_status (std_msgs/Bool)
    True  = person is present in the follow zone
    False = person has been absent for longer than person_lost_timeout

This node does NOT send velocity commands.
move_base.py reacts to this topic by cancelling/re-sending the goal.

Required: run leg_detector separately before this node:
  rosrun leg_detector leg_detector scan:=/scan \
    /opt/ros/noetic/share/leg_detector/config/trained_leg_detector.yaml
"""

import rospy
import math
import tf
import geometry_msgs.msg
from people_msgs.msg import PositionMeasurementArray
from std_msgs.msg import Bool


class PersonFollowingMonitor:
    def __init__(self):
        rospy.init_node('person_following_monitor', anonymous=True)

        # ── Parameters ───────────────────────────────────────────────────────────
        self.min_follow_dist = rospy.get_param('~min_follow_distance', 0.5)   # m
        self.max_follow_dist = rospy.get_param('~max_follow_distance', 1.5)   # m
        self.lost_timeout    = rospy.get_param('~person_lost_timeout',  3.0)  # s
        self.min_reliability = rospy.get_param('~min_reliability',      0.5)  # 0–1
        self.sensor_stale_timeout = rospy.get_param('~sensor_stale_timeout', 1.0)
        self.rear_x_threshold = rospy.get_param('~rear_x_threshold', -0.05)  # m
        self.trusted_track_duration = rospy.get_param('~trusted_track_duration', 1.5)  # s
        self.blind_spot_grace_time = rospy.get_param('~blind_spot_grace_time', 2.0)  # s

        # ── State ────────────────────────────────────────────────────────────────
        self.navigation_active     = False
        self.leg_person_present    = False
        self.last_leg_seen_time    = rospy.Time(0)
        self.current_track_start_time = rospy.Time(0)
        self.blind_spot_grace_until = rospy.Time(0)
        # last_seen_time is set properly in _nav_status_cb when nav starts
        self.last_seen_time        = rospy.Time(0)
        self.person_is_following   = False

        # ── TF ───────────────────────────────────────────────────────────────────
        self.tf_listener = tf.TransformListener()

        # ── Publishers ───────────────────────────────────────────────────────────
        self.status_pub = rospy.Publisher(
            '/movo/person_following_status', Bool, queue_size=1
        )

        # ── Subscribers ──────────────────────────────────────────────────────────
        rospy.Subscriber('/leg_tracker_measurements', PositionMeasurementArray,
                         self._leg_cb)
        rospy.Subscriber('/movo/navigation_active', Bool,
                         self._nav_status_cb)
        # Publish status at 1 Hz even when no leg messages arrive
        # (ensures the False / timeout case gets delivered promptly)
        rospy.Timer(rospy.Duration(1.0), self._timer_cb)
        self.status_pub.publish(Bool(data=False))
        rospy.loginfo(
            f"Person Following Monitor ready — "
            f"range {self.min_follow_dist}–{self.max_follow_dist} m, "
            f"timeout {self.lost_timeout} s, "
            f"min_reliability {self.min_reliability}, "
            f"sensor_stale_timeout {self.sensor_stale_timeout} s, "
            f"rear_x_threshold {self.rear_x_threshold} m, "
            f"trusted_track_duration {self.trusted_track_duration} s, "
            f"blind_spot_grace_time {self.blind_spot_grace_time} s, "
            f"lidar_only=True"
        )

    # ── Navigation state ─────────────────────────────────────────────────────

    def _nav_status_cb(self, msg):
        was_active = self.navigation_active
        self.navigation_active = msg.data

        if self.navigation_active and not was_active:
            # Navigation just STARTED — require a real detection before reporting true
            self.last_seen_time        = rospy.Time(0)
            self.person_is_following   = False
            self.leg_person_present    = False
            self.last_leg_seen_time    = rospy.Time(0)
            self.current_track_start_time = rospy.Time(0)
            self.blind_spot_grace_until = rospy.Time(0)
            rospy.loginfo("Navigation started — person following monitoring active")

        elif not self.navigation_active and was_active:
            # Navigation just STOPPED — clear state and publish false so stale
            # true values do not linger when nobody is being tracked.
            self.leg_person_present    = False
            self.last_leg_seen_time    = rospy.Time(0)
            self.current_track_start_time = rospy.Time(0)
            self.blind_spot_grace_until = rospy.Time(0)
            self.last_seen_time        = rospy.Time(0)
            self.person_is_following   = False
            self.status_pub.publish(Bool(data=False))
            rospy.loginfo("Navigation ended — person following monitoring paused")

    # ── Leg detector callback ────────────────────────────────────────────────

    def _leg_cb(self, msg):
        now = rospy.Time.now()
        found = False
        for person in msg.people:
            if person.reliability < self.min_reliability:
                continue

            try:
                # Transform the leg position into the robot's base_link frame
                pt = geometry_msgs.msg.PointStamped()
                pt.header = person.header
                pt.point  = person.pos

                self.tf_listener.waitForTransform(
                    'base_link', pt.header.frame_id,
                    pt.header.stamp, rospy.Duration(0.2)
                )
                pt_base = self.tf_listener.transformPoint('base_link', pt)

                x    = pt_base.point.x   # positive = in front, negative = behind
                y    = pt_base.point.y
                dist = math.sqrt(x ** 2 + y ** 2)
                in_range = self.min_follow_dist <= dist <= self.max_follow_dist
                behind_robot = x <= self.rear_x_threshold

                # Always log at INFO so we can see what's being detected
                rospy.loginfo_throttle(
                    1.0,
                    f"Leg at base_link ({x:.2f}, {y:.2f}) m, dist={dist:.2f}, "
                    f"reliability={person.reliability:.2f} "
                    f"[behind={behind_robot}, in_range={in_range}]"
                )

                # Person must be behind the robot and within the follow range.
                if behind_robot and in_range:
                    found = True
                    self.last_leg_seen_time = now
                    rospy.loginfo_throttle(
                        1.0,
                        f"Valid follow target at ({x:.2f}, {y:.2f}) m, dist={dist:.2f}, "
                        f"reliability={person.reliability:.2f}"
                    )
                    break

            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, f"TF error: {e}")

        if found:
            if not self.leg_person_present:
                self.current_track_start_time = now
            self.blind_spot_grace_until = rospy.Time(0)
        elif self.leg_person_present:
            tracked_duration = 0.0
            if self.current_track_start_time != rospy.Time(0):
                tracked_duration = (now - self.current_track_start_time).to_sec()
            if tracked_duration >= self.trusted_track_duration:
                self.blind_spot_grace_until = now + rospy.Duration(self.blind_spot_grace_time)
                rospy.loginfo(
                    "Stable track lost after %.1fs — applying %.1fs blind-spot grace.",
                    tracked_duration,
                    self.blind_spot_grace_time,
                )
            self.current_track_start_time = rospy.Time(0)

        self.leg_person_present = found
        # Only act on the result (pause/resume nav) when actively navigating
        if self.navigation_active:
            self._update_status()

    # ── Status update ────────────────────────────────────────────────────────

    def _update_status(self):
        """Publish following status using lidar leg detections only."""
        now = rospy.Time.now()
        leg_fresh = (
            self.leg_person_present and
            (now - self.last_leg_seen_time).to_sec() <= self.sensor_stale_timeout
        )
        blind_spot_grace_active = now <= self.blind_spot_grace_until

        if leg_fresh:
            self.last_seen_time      = now
            self.person_is_following = True
        else:
            elapsed = (now - self.last_seen_time).to_sec()
            effective_lost_timeout = self.lost_timeout
            if blind_spot_grace_active:
                effective_lost_timeout += self.blind_spot_grace_time
            if elapsed > effective_lost_timeout:
                if self.person_is_following:
                    rospy.logwarn(
                        f"Person not detected for {elapsed:.1f} s "
                        f"(legs={leg_fresh}, blind_spot_grace={blind_spot_grace_active}) "
                        f"— signalling stop"
                    )
                self.person_is_following = False
            elif blind_spot_grace_active:
                rospy.loginfo_throttle(
                    1.0,
                    "Blind-spot grace active for %.1fs more.",
                    max(0.0, (self.blind_spot_grace_until - now).to_sec()),
                )

        status      = Bool()
        status.data = self.person_is_following
        self.status_pub.publish(status)

    def _timer_cb(self, _event):
        """Periodic publish so timeouts fire even when no leg messages arrive."""
        if self.navigation_active:
            self._update_status()
        else:
            self.status_pub.publish(Bool(data=False))

    # ── Main ────────────────────────────────────────────────────────────────

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = PersonFollowingMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
