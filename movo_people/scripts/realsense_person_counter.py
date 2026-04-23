#!/usr/bin/env python3

import rospy
import cv2
import os
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

# Ensure sibling modules are importable under rosrun/catkin wrapper execution.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from yolo_person_detector import YoloPersonDetector

class RealSensePersonCounter:
    """
    ROS Node for integrating YOLOv8 detection logic with camera color stream.
    """
    def __init__(self):
        rospy.init_node('realsense_person_counter', anonymous=True)
        rospy.loginfo("RealSense Person Counter Node Initialized.")
        self.bridge = CvBridge()
        self.person_count_pub = rospy.Publisher('/people/count', Int32, queue_size=10)

        color_topic = '/camera/color/image_raw'
        self.color_sub = rospy.Subscriber(color_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribing to Color: {color_topic}")

        # Load YOLO once; model path can be swapped if you want a larger model (e.g. yolov8s.pt).
        model_path = os.path.join(SCRIPT_DIR, "yolov8n.pt")
        self.detector = YoloPersonDetector(model_path=model_path, conf=0.25)

    def process_images(self, color_image):
        try:
            person_count, annotated_img, boxes = self.detector.detect_people(color_image)

            # Optional: visualize detections locally for tuning.
            # cv2.imshow("People Detection", annotated_img)
            # cv2.waitKey(1)
            return person_count
        except Exception as e:
            rospy.logerr(f"Error in detection logic: {e}")
            return 0

    def image_callback(self, color_msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        person_count = self.process_images(color_image)
        count_msg = Int32()
        count_msg.data = person_count
        self.person_count_pub.publish(count_msg)
        rospy.loginfo(f"Published Person Count: {person_count}")
        # Plain print so output shows even if ROS logs are not visible
        try:
            stamp = color_msg.header.stamp.to_sec()
        except Exception:
            stamp = rospy.get_time()
        print(f"[{stamp:.3f}] Published Person Count: {person_count}")

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        counter = RealSensePersonCounter()
        counter.run()
    except rospy.ROSInterruptException:
        pass
