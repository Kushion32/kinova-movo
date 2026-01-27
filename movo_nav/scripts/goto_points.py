#!/usr/bin/env python3
import rospy
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class WaypointNavigator:
    def __init__(self, yaml_path):
        rospy.loginfo(f"Loading waypoint file: {yaml_path}")
        with open(yaml_path, 'r') as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.client = actionlib.SimpleActionClient('/movo_move_base', MoveBaseAction)
        rospy.loginfo("Waiting for movo_move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("movo_move_base connected.")

    def goto(self, name):
        if name not in self.waypoints:
            rospy.logerr("Unknown waypoint: %s", name)
            return

        wp = self.waypoints[name]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = wp['position'][0]
        goal.target_pose.pose.position.y = wp['position'][1]
        goal.target_pose.pose.position.z = wp['position'][2]

        # orientation is [x, y, z, w]
        goal.target_pose.pose.orientation.x = wp['orientation'][0]
        goal.target_pose.pose.orientation.y = wp['orientation'][1]
        goal.target_pose.pose.orientation.z = wp['orientation'][2]
        goal.target_pose.pose.orientation.w = wp['orientation'][3]

        rospy.loginfo(f"Sending robot to: {name}")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(f"Arrived at {name}")

if __name__ == "__main__":
    rospy.init_node("waypoint_navigator")
    print("Waypoint Navigator Started")

    yaml_path = rospy.get_param(
        "~waypoints_file",
        "/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml"
    )

    nav = WaypointNavigator(yaml_path)

    # ------------------------------
    # Interactive Terminal Loop
    # ------------------------------
    print("\nInteractive mode enabled!")
    print("Type a waypoint name to send the robot there.")
    print("Type 'list' to show available waypoints.")
    print("Type 'exit' or Ctrl+C to quit.\n")

    while not rospy.is_shutdown():
        try:
            user_input = input("Enter waypoint: ").strip()

            if user_input == "":
                continue

            if user_input.lower() == "exit":
                print("Exiting...")
                break

            if user_input.lower() == "list":
                print("Available waypoints:")
                for w in nav.waypoints.keys():
                    print(" -", w)
                print()
                continue

            nav.goto(user_input)

        except (KeyboardInterrupt, EOFError):
            print("\nExiting navigator.")
            break
