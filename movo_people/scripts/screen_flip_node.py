#!/usr/bin/env python3
"""
screen_flip_node.py – Flip MOVO's head so the BACK screen faces the nearest person.

How it works
------------
1. A ROS service `/movo/flip_to_back_screen` is called (e.g. from a button or terminal).
2. One depth frame is grabbed from the RealSense camera.
3. The depth image is split left/right; the side with the closer median depth
   is where the nearest object (person) is standing.
4. The head is panned OPPOSITE that direction so the back screen faces them,
   then tilted to BACK_SCREEN_TILT.
5. A second service `/movo/flip_to_front_screen` returns the head to home.

Terminal usage
--------------
  rosservice call /movo/flip_to_back_screen
  rosservice call /movo/flip_to_front_screen

Dependencies: cv_bridge, sensor_msgs, movo_msgs (already in catkin workspace)
"""
import math
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from movo_msgs.msg import PanTiltCmd, PVA

# ── Tunable constants ─────────────────────────────────────────────────────────

# The tilt angle (rad) that tilts the head so the BACK screen faces the person.
# MOVO's URDF tilt limits are ±π/2 (±90°).
# Try TILT_MIN first (-π/2 = chin up / head leans backward).
# If that exposes the front screen instead, swap the sign to +π/2.
BACK_SCREEN_TILT = -math.radians(45)     # −45° = head tilted down

# How far to pan sideways (rad).  π/2 = 90°.
# The node will choose ±90° based on where the person is detected.
# If the hardware limit prevents full 90°, lower this to math.radians(80).
PAN_SIDE = math.pi / 2                   # 90°

# Head home position
HOME_PAN  = 0.0
HOME_TILT = 0.0

# Motion speed
PAN_VEL  = 0.4   # rad/s
TILT_VEL = 0.4
ACC      = 0.3   # rad/s²

# How many intermediate steps to use when interpolating motion
MOTION_STEPS = 30
STEP_SLEEP_S = 0.05   # seconds between steps → ~20 Hz

# RealSense depth topic
DEPTH_TOPIC = '/camera/depth/image_rect_raw'

# Ignore depth readings beyond this distance (likely empty space/walls)
DEPTH_MAX_M = 4.0

# Fraction of the image width to treat as the centre dead-zone (no clear side)
CENTRE_BAND = 0.15   # ±15% around centre is considered ambiguous

# ─────────────────────────────────────────────────────────────────────────────


def _make_cmd(pan: float, tilt: float) -> PanTiltCmd:
    msg = PanTiltCmd()
    msg.header.stamp = rospy.Time.now()
    msg.pan_cmd  = PVA(pos_rad=float(pan),  vel_rps=PAN_VEL,  acc_rps2=ACC)
    msg.tilt_cmd = PVA(pos_rad=float(tilt), vel_rps=TILT_VEL, acc_rps2=ACC)
    return msg


def _interpolate(pub, start_pan, start_tilt, end_pan, end_tilt, steps=MOTION_STEPS):
    """Smoothly move head from start to end pose."""
    for i in range(1, steps + 1):
        t = i / steps
        pan  = start_pan  + t * (end_pan  - start_pan)
        tilt = start_tilt + t * (end_tilt - start_tilt)
        pub.publish(_make_cmd(pan, tilt))
        rospy.sleep(STEP_SLEEP_S)


def _current_head_pose():
    """Read the current pan/tilt from joint states (best-effort)."""
    from sensor_msgs.msg import JointState
    try:
        js = rospy.wait_for_message('/movo/head/joint_states', JointState, timeout=2.0)
        pan  = js.position[js.name.index('pan_joint')]  if 'pan_joint'  in js.name else 0.0
        tilt = js.position[js.name.index('tilt_joint')] if 'tilt_joint' in js.name else 0.0
        return pan, tilt
    except Exception:
        return 0.0, 0.0


class ScreenFlipNode:
    def __init__(self):
        rospy.init_node('screen_flip_node', anonymous=False)

        self._bridge = CvBridge()
        self._pub    = rospy.Publisher('/movo/head/cmd', PanTiltCmd, queue_size=10)

        # Services
        rospy.Service('/movo/flip_to_back_screen',  Trigger, self._cb_flip_back)
        rospy.Service('/movo/flip_to_front_screen', Trigger, self._cb_flip_front)

        rospy.loginfo("[screen_flip_node] Ready (depth-only mode).")
        rospy.loginfo("  rosservice call /movo/flip_to_back_screen")
        rospy.loginfo("  rosservice call /movo/flip_to_front_screen")

    # ── Depth-based direction detection ──────────────────────────────────────

    def _detect_closest_person_direction(self):
        """
        Splits the depth image into left and right halves and returns which
        side has the closest valid reading – that is where the person is.
        Returns 'left', 'right', or 'unknown'.
        """
        try:
            depth_msg = rospy.wait_for_message(DEPTH_TOPIC, Image, timeout=3.0)
        except Exception as e:
            rospy.logwarn(f"[screen_flip_node] Could not get depth image: {e} – defaulting to LEFT")
            return 'left'

        try:
            if depth_msg.encoding in ('16UC1', '16UC'):
                raw = self._bridge.imgmsg_to_cv2(depth_msg, '16UC1').astype(np.float32)
                depth = raw / 1000.0   # mm → m
            else:
                depth = self._bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except Exception as e:
            rospy.logwarn(f"[screen_flip_node] Depth decode error: {e} – defaulting to LEFT")
            return 'left'

        h, w = depth.shape[:2]
        half = w // 2
        band = int(w * CENTRE_BAND)

        # Use only valid, close-range pixels
        valid = (depth > 0.1) & (depth < DEPTH_MAX_M) & np.isfinite(depth)

        left_pixels  = depth[:, :half - band][valid[:, :half - band]]
        right_pixels = depth[:, half + band:][valid[:, half + band:]]

        left_min  = float(np.percentile(left_pixels,  5)) if left_pixels.size  > 50 else float('inf')
        right_min = float(np.percentile(right_pixels, 5)) if right_pixels.size > 50 else float('inf')

        rospy.loginfo(
            f"[screen_flip_node] Depth 5th-pct: left={left_min:.2f}m  right={right_min:.2f}m"
        )

        if left_min == float('inf') and right_min == float('inf'):
            rospy.logwarn("[screen_flip_node] No valid depth on either side – defaulting to LEFT")
            return 'left'

        direction = 'left' if left_min <= right_min else 'right'
        rospy.loginfo(f"[screen_flip_node] Closest object is to the {direction}")
        return direction

    # ── Service callbacks ────────────────────────────────────────────────────

    def _cb_flip_back(self, req):
        """
        Detect nearest person, pan OPPOSITE direction so the back screen
        faces them, then tilt to BACK_SCREEN_TILT.
        """
        rospy.loginfo("[screen_flip_node] flip_to_back_screen triggered")
        rospy.sleep(0.3)   # let publisher connect

        direction = self._detect_closest_person_direction()

        # Pan OPPOSITE to person so the back screen faces them
        if direction == 'left':
            target_pan = -PAN_SIDE   # pan right → back faces left (toward person)
        elif direction == 'right':
            target_pan = +PAN_SIDE   # pan left → back faces right (toward person)
        else:
            # Unknown → default to panning right
            target_pan = -PAN_SIDE

        target_tilt = BACK_SCREEN_TILT

        rospy.loginfo(
            f"[screen_flip_node] Person detected {direction} → "
            f"panning opposite to {math.degrees(target_pan):.0f}°, "
            f"tilting to {math.degrees(target_tilt):.0f}°"
        )

        cur_pan, cur_tilt = _current_head_pose()

        # Phase 1: pan to side first (keep current tilt)
        _interpolate(self._pub, cur_pan, cur_tilt, target_pan, cur_tilt)
        rospy.sleep(0.2)

        # Phase 2: tilt to back-screen angle
        _interpolate(self._pub, target_pan, cur_tilt, target_pan, target_tilt)

        return TriggerResponse(success=True,
                               message=f"Back screen facing {direction} (pan={math.degrees(target_pan):.0f}°)")

    def _cb_flip_front(self, req):
        """Return head to home position so the front screen is accessible."""
        rospy.loginfo("[screen_flip_node] flip_to_front_screen triggered")
        cur_pan, cur_tilt = _current_head_pose()

        # Phase 1: tilt back to zero first (avoid mechanical stress at full pan)
        _interpolate(self._pub, cur_pan, cur_tilt, cur_pan, HOME_TILT)
        rospy.sleep(0.2)

        # Phase 2: pan back to home
        _interpolate(self._pub, cur_pan, HOME_TILT, HOME_PAN, HOME_TILT)

        return TriggerResponse(success=True, message="Head returned to home (front screen)")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        ScreenFlipNode().run()
    except rospy.ROSInterruptException:
        pass
