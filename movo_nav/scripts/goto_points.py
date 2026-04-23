#!/usr/bin/env python3
"""goto_points.py — waypoint navigator with UI integration.

Topics published (latched):
  /ui_waypoints_list   std_msgs/String  — JSON array of waypoint names
  /ui_nav_ready        std_msgs/Bool    — True once move_base connected
  /ui_robot_arrived    std_msgs/String  — waypoint name on arrival (empty = cleared)

Topics subscribed:
  /ui_navigation_command  std_msgs/String  — "go:<name>" or "confirm:<name>"
"""
import os
import json
import threading
import time
import math

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


WAYPOINT_ALIASES = {
    "robotstack": "elevator",
    "bathroom": "washroom",
    "bathrooms": "washroom",
    "washrooms": "washroom",
    "storage": "stairs",
    "stair": "stairs",
    "elevators": "elevator",
}


class WaypointNavigator:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.waypoints = {}
        self._load_waypoints()

        # ── Publishers ────────────────────────────────────────────────────────
        # Create publishers BEFORE connecting to move_base so the UI sees the
        # latched topics immediately when it subscribes.
        self.waypoints_pub = rospy.Publisher(
            "/ui_waypoints_list", String, queue_size=1, latch=True
        )
        self.nav_ready_pub = rospy.Publisher(
            "/ui_nav_ready", Bool, queue_size=1, latch=True
        )
        self.arrived_pub = rospy.Publisher(
            "/ui_robot_arrived", String, queue_size=1, latch=True
        )
        self.cmd_vel_pub = rospy.Publisher(
            "/movo/teleop/cmd_vel", Twist, queue_size=10
        )
        # Tells person_following_monitor when the robot is actively driving
        self.nav_active_pub = rospy.Publisher(
            "/movo/navigation_active", Bool, queue_size=1, latch=True
        )
        rospy.sleep(0.3)  # let rosbridge discover the new topics

        # Publish initial state
        self._publish_waypoints_list()
        self.nav_ready_pub.publish(Bool(data=False))

        self.client = None
        self._nav_ready = False
        self._queued_command = None   # command received before nav was ready
        self._current_goal = None
        self._confirm_pending = None  # waypoint name waiting for user confirm
        self._home_pose = None        # pose saved before each outbound trip
        self._current_pose = None     # latest amcl_pose
        self._person_is_following = True  # updated by /movo/person_following_status
        self._person_following_active = False  # True once monitor publishes at least once
        self._ignore_person_following_near_goal_dist = rospy.get_param(
            '~ignore_person_following_near_goal_dist', 0.75
        )
        self._person_search_timeout = rospy.get_param('~person_search_timeout', 5.0)
        self._person_search_angular_speed = rospy.get_param('~person_search_angular_speed', 0.35)
        self._person_search_direction_switch_time = rospy.get_param(
            '~person_search_direction_switch_time', 1.5
        )
        self._flip_front_service_name = rospy.get_param(
            '~flip_front_service', '/movo/flip_to_front_screen'
        )
        self._auto_flip_command_topic = rospy.get_param(
            '~auto_flip_command_topic', '/movo/auto_screen_flip/command'
        )
        self._flip_front_srv = None

        # Fallback for setups using auto_screen_flip_node instead of service-based node.
        self._auto_flip_cmd_pub = rospy.Publisher(
            self._auto_flip_command_topic, String, queue_size=1
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber(
            "/ui_navigation_command", String, self._ui_callback, queue_size=10
        )
        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped,
            self._amcl_callback, queue_size=1
        )
        rospy.Subscriber(
            "/movo/person_following_status", Bool,
            self._person_following_cb, queue_size=1
        )

        # ── Background threads ────────────────────────────────────────────────
        threading.Thread(target=self._connect_move_base, daemon=True).start()
        threading.Thread(target=self._watch_waypoints_file, daemon=True).start()

        rospy.loginfo("WaypointNavigator ready (connecting to move_base in background).")

    # ── Waypoint file helpers ─────────────────────────────────────────────────

    def _load_waypoints(self):
        try:
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
            self.waypoints = data.get("waypoints", {})
            rospy.loginfo("Loaded %d waypoints from %s", len(self.waypoints), self.yaml_path)
        except Exception as e:
            rospy.logwarn("Could not load waypoints file: %s", e)
            self.waypoints = {}

    def _publish_waypoints_list(self):
        names = sorted(self.waypoints.keys())
        self.waypoints_pub.publish(String(data=json.dumps(names)))
        rospy.loginfo("Published waypoints list: %s", names)

    def _watch_waypoints_file(self):
        """Background thread: reload waypoints when the YAML file changes."""
        last_mtime = None
        while not rospy.is_shutdown():
            try:
                mtime = os.path.getmtime(self.yaml_path)
                if last_mtime is not None and mtime != last_mtime:
                    rospy.loginfo("Waypoints file changed — reloading.")
                    self._load_waypoints()
                    self._publish_waypoints_list()
                last_mtime = mtime
            except OSError:
                pass
            time.sleep(2.0)

    # ── Pose tracking ─────────────────────────────────────────────────────────

    def _amcl_callback(self, msg):
        self._current_pose = msg.pose.pose

    def _person_following_cb(self, msg):
        self._person_following_active = True  # monitor is running
        self._person_is_following = msg.data
        if not msg.data:
            rospy.logwarn_throttle(5.0, "Person not following — robot will pause at next loop check.")

    # ── move_base connection ──────────────────────────────────────────────────

    def _connect_move_base(self):
        """Background thread: try action servers in order, use first that responds."""
        servers = [
            ("move_base_navi", 30),   # remapped move_base — comes up fastest
            ("/movo_move_base",  60), # wrapper — needs move_base_navi first
            ("move_base",        30), # fallback standard name
        ]
        for server_name, timeout_sec in servers:
            if rospy.is_shutdown():
                return
            rospy.loginfo("Trying action server %s (up to %ds)…", server_name, timeout_sec)
            client = actionlib.SimpleActionClient(server_name, MoveBaseAction)
            if client.wait_for_server(timeout=rospy.Duration(timeout_sec)):
                rospy.loginfo("Connected to %s.", server_name)
                self.client = client
                self._nav_ready = True
                self.nav_ready_pub.publish(Bool(data=True))
                if self._queued_command:
                    cmd = self._queued_command
                    self._queued_command = None
                    rospy.loginfo("Executing queued command: %s", cmd)
                    self._dispatch(cmd)
                return
            rospy.logwarn("  %s not available, trying next…", server_name)

        rospy.logerr("No move_base action server found. Nav will remain disabled.")

    # ── UI command handler ────────────────────────────────────────────────────

    def _ui_callback(self, msg):
        cmd = msg.data.strip()
        rospy.loginfo("UI command: %s", cmd)
        if not self._nav_ready:
            rospy.loginfo("Nav not ready — queuing: %s", cmd)
            self._queued_command = cmd
            return
        self._dispatch(cmd)

    def _dispatch(self, cmd):
        if cmd.startswith("confirm:"):
            raw = cmd[len("confirm:"):].strip()
            name = self._resolve_waypoint_name(raw)
            self._handle_confirm(name)
        elif cmd.startswith("go:"):
            raw = cmd[len("go:"):].strip()
            name = self._resolve_waypoint_name(raw)
            threading.Thread(target=self._execute_goto, args=(name,), daemon=True).start()
        else:
            # Legacy plain waypoint name
            name = self._resolve_waypoint_name(cmd)
            threading.Thread(target=self._execute_goto, args=(name,), daemon=True).start()

    def _resolve_waypoint_name(self, name):
        """Resolve legacy/case-variant UI names to canonical waypoint keys."""
        candidate = (name or "").strip()
        if not candidate:
            return candidate
        if candidate in self.waypoints:
            return candidate

        # Case-insensitive direct match.
        lower_to_original = {k.lower(): k for k in self.waypoints.keys()}
        lowered = candidate.lower()
        if lowered in lower_to_original:
            return lower_to_original[lowered]

        # Alias mapping for older UI waypoint names.
        alias = WAYPOINT_ALIASES.get(lowered)
        if alias and alias in self.waypoints:
            rospy.loginfo("Waypoint alias resolved: %s -> %s", candidate, alias)
            return alias
        if alias and alias.lower() in lower_to_original:
            resolved = lower_to_original[alias.lower()]
            rospy.loginfo("Waypoint alias resolved: %s -> %s", candidate, resolved)
            return resolved

        return candidate

    def _distance_to_goal(self, wp):
        if self._current_pose is None:
            return None
        dx = wp["position"][0] - self._current_pose.position.x
        dy = wp["position"][1] - self._current_pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _search_for_person(self, name):
        rospy.loginfo(
            "Person lost — searching for up to %.1fs before resuming %s.",
            self._person_search_timeout,
            name,
        )
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        next_switch_time = start_time + rospy.Duration(self._person_search_direction_switch_time)
        direction = 1.0
        twist = Twist()

        while not rospy.is_shutdown():
            if self._person_is_following:
                self._stop_robot()
                rospy.loginfo("Person detected again — resuming navigation to %s", name)
                return True

            now = rospy.Time.now()
            if (now - start_time).to_sec() >= self._person_search_timeout:
                break

            if now >= next_switch_time:
                direction *= -1.0
                next_switch_time = now + rospy.Duration(self._person_search_direction_switch_time)

            twist.angular.z = direction * self._person_search_angular_speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._stop_robot()
        return self._person_is_following

    def _execute_goto(self, name):
        """Navigate to waypoint, then publish arrival."""
        if name not in self.waypoints:
            rospy.logerr("Unknown waypoint: %s", name)
            self.arrived_pub.publish(String(data=""))
            return

        wp = self.waypoints[name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["position"][0]
        goal.target_pose.pose.position.y = wp["position"][1]
        goal.target_pose.pose.position.z = wp["position"][2]
        goal.target_pose.pose.orientation.x = wp["orientation"][0]
        goal.target_pose.pose.orientation.y = wp["orientation"][1]
        goal.target_pose.pose.orientation.z = wp["orientation"][2]
        goal.target_pose.pose.orientation.w = wp["orientation"][3]

        # Save current position as home before departing
        if self._current_pose is not None:
            self._home_pose = self._current_pose
            rospy.loginfo("Saved home pose: (%.2f, %.2f)",
                          self._home_pose.position.x, self._home_pose.position.y)

        rospy.loginfo("Sending robot to: %s", name)
        self._current_goal = name
        self._confirm_pending = name
        self.arrived_pub.publish(String(data=""))   # clear previous arrival state

        # Person-following pause/resume loop.
        # Only active when the person_following_monitor node is actually running
        # (i.e. it has published at least one message on /movo/person_following_status).
        # If the monitor is not running, navigate straight through without any checks.
        PERSON_LOST_ABORT_TIMEOUT = 30.0  # give up waiting after this many seconds
        state = None

        if not self._person_following_active:
            # ── Simple navigation (no person following) ───────────────────────
            rospy.loginfo("Person following monitor not active — navigating normally.")
            self.nav_active_pub.publish(Bool(data=True))
            self.client.send_goal(goal)
            self.client.wait_for_result()
            state = self.client.get_state()
        else:
            # ── Person-following pause/resume loop ────────────────────────────
            ignore_person_following = False
            while not rospy.is_shutdown():
                self.nav_active_pub.publish(Bool(data=True))
                self.client.send_goal(goal)
                # Poll until goal finishes or person is lost
                while not rospy.is_shutdown():
                    state = self.client.get_state()
                    if state in (3, 4, 5):  # SUCCEEDED / ABORTED / REJECTED
                        break
                    distance_to_goal = self._distance_to_goal(wp)
                    if (
                        not ignore_person_following and
                        distance_to_goal is not None and
                        distance_to_goal <= self._ignore_person_following_near_goal_dist
                    ):
                        ignore_person_following = True
                        rospy.loginfo(
                            "Within %.2fm of %s — ignoring person following for final alignment.",
                            self._ignore_person_following_near_goal_dist,
                            name,
                        )
                    if not ignore_person_following and not self._person_is_following:
                        rospy.logwarn("Person lost — cancelling goal, waiting for them to return.")
                        self.client.cancel_goal()
                        rospy.sleep(0.3)  # let cancel propagate
                        break
                    rospy.sleep(0.2)

                # If goal completed (success or real failure), exit the loop
                if state in (3, 4, 5):
                    break

                # Person was lost — actively search before giving up.
                deadline = rospy.Time.now() + rospy.Duration(PERSON_LOST_ABORT_TIMEOUT)
                while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                    if self._search_for_person(name):
                        goal.target_pose.header.stamp = rospy.Time.now()  # refresh timestamp
                        break
                    rospy.sleep(0.2)
                else:
                    rospy.logwarn(
                        "Person not found after %.0fs of search attempts — aborting navigation.",
                        PERSON_LOST_ABORT_TIMEOUT,
                    )
                    state = 4  # treat as ABORTED
                    break

        self.nav_active_pub.publish(Bool(data=False))
        if state == 3:
            rospy.loginfo("Arrived at %s", name)
            self.arrived_pub.publish(String(data=name))
        else:
            rospy.logwarn("Navigation to %s ended with state %s", name, state)
            self.arrived_pub.publish(String(data=""))
        self._current_goal = None

    def _handle_confirm(self, name):
        """User confirmed arrival — navigate robot back to home position."""
        rospy.loginfo("User confirmed arrival at: %s — returning home.", name)
        self._confirm_pending = None
        self._request_head_home()
        # Tell the UI the robot is now on its way back (keeps the indicator visible)
        self.arrived_pub.publish(String(data="returning"))
        threading.Thread(target=self._return_home, daemon=True).start()

    def _request_head_home(self):
        """Return the tablet/head to front when user sends robot back."""
        try:
            rospy.wait_for_service(self._flip_front_service_name, timeout=0.6)
            if self._flip_front_srv is None:
                self._flip_front_srv = rospy.ServiceProxy(self._flip_front_service_name, Trigger)
            resp = self._flip_front_srv()
            if getattr(resp, 'success', False):
                rospy.loginfo("Head reset via service: %s", self._flip_front_service_name)
            else:
                rospy.logwarn(
                    "Head reset service returned failure (%s): %s",
                    self._flip_front_service_name,
                    getattr(resp, 'message', ''),
                )
            return
        except Exception as exc:
            rospy.logwarn(
                "Head reset service unavailable (%s): %s. Falling back to topic command.",
                self._flip_front_service_name,
                exc,
            )

        # Fallback path for auto_screen_flip_node.
        self._auto_flip_cmd_pub.publish(String(data='home'))
        rospy.loginfo("Head reset command published to %s", self._auto_flip_command_topic)

    def _return_home(self):
        """Navigate back to the position saved before the last outbound trip.
        Falls back to the 'home' waypoint in the YAML if no AMCL pose was captured."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if self._home_pose is not None:
            goal.target_pose.pose = self._home_pose
            rospy.loginfo("Returning to saved home pose: (%.2f, %.2f)",
                          self._home_pose.position.x, self._home_pose.position.y)
        elif "home" in self.waypoints:
            wp = self.waypoints["home"]
            goal.target_pose.pose.position.x    = wp["position"][0]
            goal.target_pose.pose.position.y    = wp["position"][1]
            goal.target_pose.pose.position.z    = wp["position"][2]
            goal.target_pose.pose.orientation.x = wp["orientation"][0]
            goal.target_pose.pose.orientation.y = wp["orientation"][1]
            goal.target_pose.pose.orientation.z = wp["orientation"][2]
            goal.target_pose.pose.orientation.w = wp["orientation"][3]
            rospy.loginfo("No saved home pose — navigating to 'home' waypoint.")
        else:
            rospy.logwarn("No home pose and no 'home' waypoint — robot will stay put.")
            self.arrived_pub.publish(String(data=""))
            return

        self.client.send_goal(goal)
        self.client.wait_for_result()
        state = self.client.get_state()
        if state == 3:
            rospy.loginfo("Returned home successfully.")
        else:
            rospy.logwarn("Return home ended with state %s", state)
        # Clear the returning state on the UI
        self.arrived_pub.publish(String(data=""))


if __name__ == "__main__":
    rospy.init_node("waypoint_navigator")
    rospy.loginfo("Waypoint Navigator starting…")

    yaml_path = rospy.get_param(
        "~waypoints_file",
        "/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml",
    )

    nav = WaypointNavigator(yaml_path)
    rospy.spin()

