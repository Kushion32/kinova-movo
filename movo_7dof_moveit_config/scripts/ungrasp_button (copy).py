#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Topic that MOVO's base driver listens to (from rostopic info)
BASE_TOPIC = "/movo/cmd_vel"

# Key mappings: key -> (linear_x, angular_z)
MOVE_BINDINGS = {
    'w': ( 0.25,  0.0),   # forward
    's': (-0.25,  0.0),   # backward
    'a': ( 0.0,   0.6),   # rotate left (CCW)
    'd': ( 0.0,  -0.6),   # rotate right (CW)
    'q': ( 0.25,  0.6),   # forward-left
    'e': ( 0.25, -0.6),   # forward-right
    'z': (-0.25, 0.6),    # backward-left
    'c': (-0.25,-0.6),    # backward-right
    'x': ( 0.0,   0.0),   # stop
    ' ': ( 0.0,   0.0),   # stop (space)
}

INSTRUCTIONS = """
WASD keyboard teleop for MOVO base
----------------------------------
Controls:
    w : forward
    s : backward
    a : rotate left (CCW)
    d : rotate right (CW)

    q : forward + left
    e : forward + right
    z : backward + left
    c : backward + right

    x or SPACE : stop

    CTRL-C : quit

Publishing Twist messages on: {topic}
""".format(topic=BASE_TOPIC)


def get_key(settings, timeout=0.1):
    """
    Non-blocking key reader.
    Returns '' if no key was pressed within timeout.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("movo_wasd_teleop")
    pub = rospy.Publisher(BASE_TOPIC, Twist, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    last_cmd = Twist()  # keep last command so it keeps moving until stopped

    print(INSTRUCTIONS)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                lin_x, ang_z = MOVE_BINDINGS[key]
                last_cmd.linear.x = lin_x
                last_cmd.angular.z = ang_z
                rospy.loginfo("Key: %r -> lin_x=%.2f, ang_z=%.2f", key, lin_x, ang_z)
            elif key == '\x03':  # CTRL-C
                break

            # Publish last command every cycle
            pub.publish(last_cmd)
            rate.sleep()

    except Exception as e:
        rospy.logerr("Exception in teleop: %s", e)

    finally:
        # Make sure robot stops
        stop = Twist()
        pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nTeleop terminated, robot commanded to stop.")


if __name__ == "__main__":
    main()

