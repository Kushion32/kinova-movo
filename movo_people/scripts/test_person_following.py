#!/usr/bin/env python
"""
Test/Debug script for Person Following Monitor

This script helps you test and visualize the person following system.
Run this to see real-time status of person detection.
"""

import rospy
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class PersonFollowingDebugger:
    def __init__(self):
        rospy.init_node('person_following_debugger', anonymous=True)
        print("\n" + "="*60)
        print("Person Following Monitor - Debug Tool")
        print("="*60)
        print("This tool shows real-time status of person detection\n")
        
        self.person_following = None
        self.person_count = None
        self.navigation_active = None
        self.last_scan_time = None
        
        # Subscribe to relevant topics
        rospy.Subscriber('/movo/person_following_status', Bool, self.person_status_callback)
        rospy.Subscriber('/people/count', Int32, self.person_count_callback)
        rospy.Subscriber('/movo/navigation_active', Bool, self.nav_status_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        print("Waiting for data...")
        print("-"*60)
        
    def person_status_callback(self, msg):
        self.person_following = msg.data
        self.print_status()
        
    def person_count_callback(self, msg):
        self.person_count = msg.data
        
    def nav_status_callback(self, msg):
        self.navigation_active = msg.data
        self.print_status()
        
    def scan_callback(self, msg):
        self.last_scan_time = rospy.Time.now()
        
    def print_status(self):
        status_lines = []
        status_lines.append("\n" + "="*60)
        status_lines.append(f"Timestamp: {rospy.Time.now().to_sec():.2f}")
        status_lines.append("-"*60)
        
        # Person following status
        if self.person_following is not None:
            status = "✓ FOLLOWING" if self.person_following else "✗ NOT FOLLOWING"
            color_code = "\033[92m" if self.person_following else "\033[91m"
            reset_code = "\033[0m"
            status_lines.append(f"Person Status: {color_code}{status}{reset_code}")
        else:
            status_lines.append("Person Status: Waiting for data...")
            
        # Camera person count
        if self.person_count is not None:
            status_lines.append(f"Camera Detected: {self.person_count} person(s)")
        else:
            status_lines.append("Camera Detected: Waiting for data...")
            
        # Navigation status
        if self.navigation_active is not None:
            nav_status = "ACTIVE" if self.navigation_active else "INACTIVE"
            status_lines.append(f"Navigation: {nav_status}")
        else:
            status_lines.append("Navigation: Waiting for data...")
            
        # Laser scan status
        if self.last_scan_time is not None:
            time_since = (rospy.Time.now() - self.last_scan_time).to_sec()
            if time_since < 1.0:
                status_lines.append("Laser Scan: ✓ Active")
            else:
                status_lines.append(f"Laser Scan: ! Stale ({time_since:.1f}s ago)")
        else:
            status_lines.append("Laser Scan: Waiting for data...")
            
        status_lines.append("="*60)
        
        # Clear previous output and print
        print("\033[2J\033[H")  # Clear screen and move cursor to top
        print("\n".join(status_lines))
        
        # Print instructions
        print("\nInstructions:")
        print("  - Person should be 0.5-2.5 meters from robot")
        print("  - Robot checks behind and around (not just front)")
        print("  - If 'NOT FOLLOWING', robot will stop and turn")
        print("  - Press Ctrl+C to exit")
        print()
        
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        debugger = PersonFollowingDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        print("\nDebugger stopped.")
    except KeyboardInterrupt:
        print("\nDebugger stopped by user.")
