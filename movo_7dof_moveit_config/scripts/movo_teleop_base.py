#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# OPTION 1 (preferred path through mux):
# BASE_TOPIC = "/movo/teleop/cmd_vel"

# OPTION 2 (direct to driver, bypass mux):
BASE_TOPIC = "/movo/cmd_vel"

MOVE_BINDINGS = {
    'w': ( 0.25,  0.0),   # forward
    's': (-0.25,  0.0),   # backward
    'a': ( 0.0,   0.6),   # rotate left
    'd': ( 0.0,  -0.6),   # rotate right
    'x': ( 0.0,   0.0),   # stop
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("movo_wasd_teleop")
    pub = rospy.Publisher(BASE_TOPIC, Twist, queue_size=1)
    rate = rospy.Rate(10)

    twist = Twist()
    print(f"Publishing Twist commands to {BASE_TOPIC}")
    print("Controls: w/s = forward/back, a/d = rotate, x = stop, CTRL-C = quit")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in MOVE_BINDINGS:
                lin_x, ang_z = MOVE_BINDINGS[key]
                twist.linear.x = lin_x
                twist.angular.z = ang_z
                rospy.loginfo(twist)
                pub.publish(twist)
            elif key == '\x03':  # CTRL-C
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
