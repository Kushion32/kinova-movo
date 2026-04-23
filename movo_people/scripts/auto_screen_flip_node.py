#!/usr/bin/env python3
"""
auto_screen_flip_node.py

Continuously watches the RealSense depth image and automatically flips MOVO's
head so the back screen faces a nearby person.

Behavior:
- Default pose keeps the depth camera forward (home pan/tilt).
- Requires continuous detection for a hold time (default 5.0 s) before flipping.
- If hold completes, pans opposite that side and tilts to back-screen tilt.
- Once flipped, stays flipped until UI/user sends a command to return home.

This node uses the same head command interface as screen_flip_node.py:
  /movo/head/cmd (movo_msgs/PanTiltCmd)
"""

import math
import time

import numpy as np
import rospy
from cv_bridge import CvBridge
from movo_msgs.msg import PanTiltCmd, PVA
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String


class AutoScreenFlipNode:
    def __init__(self):
        rospy.init_node("auto_screen_flip_node", anonymous=False)

        # Topics
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self.head_cmd_topic = rospy.get_param("~head_cmd_topic", "/movo/head/cmd")
        self.head_state_topic = rospy.get_param("~head_state_topic", "/movo/head/joint_states")
        self.command_topic = rospy.get_param("~command_topic", "/movo/auto_screen_flip/command")
        self.state_topic = rospy.get_param("~state_topic", "/movo/auto_screen_flip/state")

        # Distances and detection tuning
        self.trigger_distance_m = rospy.get_param("~trigger_distance_m", 0.40)
        self.clear_distance_m = rospy.get_param("~clear_distance_m", 0.65)
        self.detect_hold_s = rospy.get_param("~detect_hold_s", 3.0)
        self.depth_min_m = rospy.get_param("~depth_min_m", 0.10)
        self.depth_max_m = rospy.get_param("~depth_max_m", 4.00)
        self.centre_band = rospy.get_param("~centre_band", 0.15)
        self.min_valid_pixels = int(rospy.get_param("~min_valid_pixels", 60))

        # Motion and pose
        self.back_screen_tilt = float(rospy.get_param("~back_screen_tilt_deg", -45.0)) * math.pi / 180.0
        self.pan_side = float(rospy.get_param("~pan_side_deg", 90.0)) * math.pi / 180.0
        self.home_pan = 0.0
        self.home_tilt = 0.0
        self.pan_vel = rospy.get_param("~pan_vel", 0.40)
        self.tilt_vel = rospy.get_param("~tilt_vel", 0.40)
        self.acc = rospy.get_param("~acc", 0.30)
        self.motion_steps = int(rospy.get_param("~motion_steps", 25))
        self.motion_step_sleep_s = rospy.get_param("~motion_step_sleep_s", 0.05)

        # State management
        self.flip_cooldown_s = rospy.get_param("~flip_cooldown_s", 2.0)
        self.loop_hz = rospy.get_param("~loop_hz", 5.0)

        self._bridge = CvBridge()
        self._head_pub = rospy.Publisher(self.head_cmd_topic, PanTiltCmd, queue_size=10)
        self._state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)

        self._latest_depth_msg = None
        self._last_flip_ts = 0.0
        self._candidate_side = None
        self._candidate_start_ts = None

        self._mode = "front"  # front | waiting_left | waiting_right | back_left | back_right
        self._state_pub.publish(String(data=self._mode))

        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=1)
        rospy.Subscriber(self.command_topic, String, self._cmd_cb, queue_size=5)

        rospy.sleep(0.3)
        self._move_home()
        rospy.loginfo("[auto_screen_flip_node] Ready. Watching %s", self.depth_topic)

    def _depth_cb(self, msg):
        self._latest_depth_msg = msg

    def _cmd_cb(self, msg):
        cmd = (msg.data or "").strip().lower()
        if not cmd:
            return

        if cmd in ("home", "front", "reset", "return_home"):
            rospy.loginfo("[auto_screen_flip_node] Command received: %s", cmd)
            self._move_home()
            self._candidate_side = None
            self._candidate_start_ts = None
            return

        if cmd in ("flip_left", "left"):
            rospy.loginfo("[auto_screen_flip_node] Command received: %s", cmd)
            self._move_to_back_for_side("left")
            return

        if cmd in ("flip_right", "right"):
            rospy.loginfo("[auto_screen_flip_node] Command received: %s", cmd)
            self._move_to_back_for_side("right")
            return

        rospy.logwarn("[auto_screen_flip_node] Unknown command: %s", cmd)

    def _make_cmd(self, pan, tilt):
        msg = PanTiltCmd()
        msg.header.stamp = rospy.Time.now()
        msg.pan_cmd = PVA(pos_rad=float(pan), vel_rps=float(self.pan_vel), acc_rps2=float(self.acc))
        msg.tilt_cmd = PVA(pos_rad=float(tilt), vel_rps=float(self.tilt_vel), acc_rps2=float(self.acc))
        return msg

    def _current_head_pose(self):
        try:
            js = rospy.wait_for_message(self.head_state_topic, JointState, timeout=1.0)
            pan = js.position[js.name.index("pan_joint")] if "pan_joint" in js.name else 0.0
            tilt = js.position[js.name.index("tilt_joint")] if "tilt_joint" in js.name else 0.0
            return pan, tilt
        except Exception:
            return 0.0, 0.0

    def _interpolate(self, start_pan, start_tilt, end_pan, end_tilt):
        for i in range(1, self.motion_steps + 1):
            t = float(i) / float(self.motion_steps)
            pan = start_pan + t * (end_pan - start_pan)
            tilt = start_tilt + t * (end_tilt - start_tilt)
            self._head_pub.publish(self._make_cmd(pan, tilt))
            rospy.sleep(self.motion_step_sleep_s)

    def _move_home(self):
        cur_pan, cur_tilt = self._current_head_pose()
        self._interpolate(cur_pan, cur_tilt, cur_pan, self.home_tilt)
        rospy.sleep(0.1)
        self._interpolate(cur_pan, self.home_tilt, self.home_pan, self.home_tilt)
        self._mode = "front"
        self._state_pub.publish(String(data=self._mode))
        rospy.loginfo("[auto_screen_flip_node] Back to front/home pose")

    def _move_to_back_for_side(self, person_side):
        # Pan opposite side so the back screen faces the person.
        if person_side == "left":
            target_pan = -self.pan_side
            mode = "back_left"
        else:
            target_pan = self.pan_side
            mode = "back_right"

        cur_pan, cur_tilt = self._current_head_pose()
        self._interpolate(cur_pan, cur_tilt, target_pan, cur_tilt)
        rospy.sleep(0.1)
        self._interpolate(target_pan, cur_tilt, target_pan, self.back_screen_tilt)

        self._mode = mode
        self._state_pub.publish(String(data=self._mode))
        self._last_flip_ts = time.time()
        rospy.loginfo(
            "[auto_screen_flip_node] Triggered: person=%s -> pan=%.1f deg tilt=%.1f deg",
            person_side,
            math.degrees(target_pan),
            math.degrees(self.back_screen_tilt),
        )

    def _decode_depth(self, depth_msg):
        if depth_msg.encoding in ("16UC1", "16UC"):
            raw = self._bridge.imgmsg_to_cv2(depth_msg, "16UC1").astype(np.float32)
            return raw / 1000.0
        return self._bridge.imgmsg_to_cv2(depth_msg, "32FC1")

    def _detect_side_and_distance(self):
        if self._latest_depth_msg is None:
            return "unknown", float("inf")

        try:
            depth = self._decode_depth(self._latest_depth_msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[auto_screen_flip_node] Depth decode failed: %s", str(exc))
            return "unknown", float("inf")

        if depth is None or len(depth.shape) < 2:
            return "unknown", float("inf")

        _, w = depth.shape[:2]
        half = w // 2
        band = int(w * self.centre_band)

        valid = (
            np.isfinite(depth)
            & (depth > self.depth_min_m)
            & (depth < self.depth_max_m)
        )

        left_mask = valid[:, : max(1, half - band)]
        right_mask = valid[:, min(w, half + band) :]

        left_pixels = depth[:, : max(1, half - band)][left_mask]
        right_pixels = depth[:, min(w, half + band) :][right_mask]

        left_dist = float(np.percentile(left_pixels, 5)) if left_pixels.size >= self.min_valid_pixels else float("inf")
        right_dist = float(np.percentile(right_pixels, 5)) if right_pixels.size >= self.min_valid_pixels else float("inf")

        nearest = min(left_dist, right_dist)
        if not np.isfinite(nearest):
            return "unknown", float("inf")

        side = "left" if left_dist <= right_dist else "right"
        return side, nearest

    def spin(self):
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            side, nearest = self._detect_side_and_distance()
            now = time.time()

            detected = (
                side in ("left", "right")
                and np.isfinite(nearest)
                and nearest <= self.trigger_distance_m
            )

            # Only auto-detect while front-facing. Once flipped, wait for UI command.
            if self._mode in ("front", "waiting_left", "waiting_right"):
                if detected:
                    if self._candidate_start_ts is None:
                        self._candidate_side = side
                        self._candidate_start_ts = now
                        self._mode = "waiting_left" if side == "left" else "waiting_right"
                        self._state_pub.publish(String(data=self._mode))
                        rospy.loginfo(
                            "[auto_screen_flip_node] Candidate %s detected at %.2fm; waiting %.1fs",
                            side,
                            nearest,
                            self.detect_hold_s,
                        )
                    else:
                        # Keep the waiting side aligned to the latest observation
                        # while preserving the same hold timer.
                        if self._candidate_side != side:
                            self._candidate_side = side
                            next_mode = "waiting_left" if side == "left" else "waiting_right"
                            if self._mode != next_mode:
                                self._mode = next_mode
                                self._state_pub.publish(String(data=self._mode))

                        held = now - self._candidate_start_ts
                        if (
                            held >= self.detect_hold_s
                            and (now - self._last_flip_ts) >= self.flip_cooldown_s
                        ):
                            # Decide direction from a final, immediate re-check.
                            final_side, final_dist = self._detect_side_and_distance()
                            if final_side not in ("left", "right") or not np.isfinite(final_dist):
                                final_side = self._candidate_side if self._candidate_side in ("left", "right") else "left"
                            self._move_to_back_for_side(final_side)
                            self._candidate_side = None
                            self._candidate_start_ts = None
                else:
                    # Allow a little hysteresis before resetting candidate timer.
                    if (
                        self._candidate_start_ts is not None
                        and np.isfinite(nearest)
                        and nearest <= self.clear_distance_m
                        and side in ("left", "right")
                    ):
                        pass
                    else:
                        self._candidate_side = None
                        self._candidate_start_ts = None
                        if self._mode.startswith("waiting_"):
                            self._mode = "front"
                            self._state_pub.publish(String(data=self._mode))

            rospy.loginfo_throttle(
                2.0,
                "[auto_screen_flip_node] mode=%s side=%s nearest=%.2fm trigger=%.2fm hold=%.1fs cmd=%s",
                self._mode,
                side,
                nearest if np.isfinite(nearest) else -1.0,
                self.trigger_distance_m,
                self.detect_hold_s,
                self.command_topic,
            )
            rate.sleep()


def main():
    node = AutoScreenFlipNode()
    node.spin()


if __name__ == "__main__":
    main()
