#!/usr/bin/env python3
"""
head_sweep.py  –  Interactive keyboard control of MOVO's head pan/tilt.

Controls:
  a / LEFT  arrow  – pan left
  d / RIGHT arrow  – pan right
  w / UP    arrow  – tilt up
  s / DOWN  arrow  – tilt down
  r                – snap back to home (0, 0)
  q / ESC          – quit

Hold a key to keep moving; the head stops when you release.

Usage:
    source /home/krish/catkin_ws/devel/setup.bash
    python3 head_sweep.py
"""
import math
import sys
import termios
import tty
import select
import rospy
from movo_msgs.msg import PanTiltCmd, PVA

# ── Limits ────────────────────────────────────────────────────────────────────
# URDF soft limits are ±90°.  Physical servo may support ±180°.
# Increase PAN_MAX / TILT_MAX if the hardware allows it.
PAN_MAX  = math.radians(180)
PAN_MIN  = -math.radians(180)
TILT_MAX = math.radians(90)
TILT_MIN = -math.radians(45)

# ── Motion parameters ─────────────────────────────────────────────────────────
PAN_STEP  = math.radians(2)   # degrees moved per key-press cycle
TILT_STEP = math.radians(2)
PAN_VEL   = 0.5               # rad/s
TILT_VEL  = 0.5
ACC       = 0.5               # rad/s²
LOOP_HZ   = 20                # control loop rate


def make_cmd(pan, tilt):
    msg = PanTiltCmd()
    msg.header.stamp = rospy.Time.now()
    msg.pan_cmd  = PVA(pos_rad=float(pan),  vel_rps=PAN_VEL,  acc_rps2=ACC)
    msg.tilt_cmd = PVA(pos_rad=float(tilt), vel_rps=TILT_VEL, acc_rps2=ACC)
    return msg


def get_key(timeout=0.05):
    """Return a single keypress (non-blocking, waits up to `timeout` secs).
    Returns '' if no key was pressed."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            # Escape sequences for arrow keys: ESC [ A/B/C/D
            if ch == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        rlist3, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if rlist3:
                            ch3 = sys.stdin.read(1)
                            return 'UP'    if ch3 == 'A' else \
                                   'DOWN'  if ch3 == 'B' else \
                                   'RIGHT' if ch3 == 'C' else \
                                   'LEFT'  if ch3 == 'D' else ''
                return 'ESC'
            return ch
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_head_state():
    """Read current pan/tilt from joint states."""
    from sensor_msgs.msg import JointState
    try:
        js = rospy.wait_for_message('/movo/head/joint_states', JointState, timeout=2.0)
        pan  = js.position[js.name.index('pan_joint')]  if 'pan_joint'  in js.name else 0.0
        tilt = js.position[js.name.index('tilt_joint')] if 'tilt_joint' in js.name else 0.0
        return pan, tilt
    except Exception:
        return 0.0, 0.0


def main():
    rospy.init_node('head_keyboard', anonymous=True)
    pub = rospy.Publisher('/movo/head/cmd', PanTiltCmd, queue_size=10)
    rospy.sleep(0.5)

    pan, tilt = read_head_state()
    rate = rospy.Rate(LOOP_HZ)

    print("\n=== MOVO Head Keyboard Control ===")
    print("  a / ←  : pan left      d / →  : pan right")
    print("  w / ↑  : tilt up       s / ↓  : tilt down")
    print("  r      : return home   q / ESC: quit")
    print(f"  Pan range: [{math.degrees(PAN_MIN):.0f}°, {math.degrees(PAN_MAX):.0f}°]")
    print(f"  Tilt range: [{math.degrees(TILT_MIN):.0f}°, {math.degrees(TILT_MAX):.0f}°]")
    print("===================================\n")

    while not rospy.is_shutdown():
        key = get_key(timeout=1.0 / LOOP_HZ)

        if key in ('q', 'ESC'):
            print("\nQuitting — returning head to home.")
            # Glide back to home
            steps = max(int(max(abs(pan), abs(tilt)) / math.radians(2)), 1)
            for i in range(steps + 1):
                t = i / steps
                pub.publish(make_cmd(pan * (1 - t), tilt * (1 - t)))
                rospy.sleep(0.05)
            break

        moved = False
        if key in ('a', 'LEFT'):
            pan = min(pan + PAN_STEP, PAN_MAX);  moved = True
        elif key in ('d', 'RIGHT'):
            pan = max(pan - PAN_STEP, PAN_MIN);  moved = True
        elif key in ('w', 'UP'):
            tilt = min(tilt + TILT_STEP, TILT_MAX); moved = True
        elif key in ('s', 'DOWN'):
            tilt = max(tilt - TILT_STEP, TILT_MIN); moved = True
        elif key == 'r':
            pan, tilt = 0.0, 0.0; moved = True

        if moved:
            pub.publish(make_cmd(pan, tilt))
            print(f"\r  pan={math.degrees(pan):+.1f}°  tilt={math.degrees(tilt):+.1f}°    ", end='', flush=True)

        rate.sleep()

    print("\nDone.")


if __name__ == '__main__':
    main()
