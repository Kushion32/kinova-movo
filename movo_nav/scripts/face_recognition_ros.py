#!/usr/bin/env python3

import rospy
import cv2
import face_recognition
import sys
import os
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# ----------------------------
# ROS Face Authentication Node
# ----------------------------
class FaceAuthNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Load parameters
        self.display = rospy.get_param("~display", True)
        
        known_image_path = rospy.get_param(
            "~known_image_path",
            os.path.expanduser("~/me.jpg")
        )

        camera_topic = rospy.get_param(
            "~camera_topic",
            "/camera/color/image_raw/compressed"
        )

        auth_status_topic = rospy.get_param(
            "~auth_status_topic",
            "/face_auth/status"
        )
        auth_overlay_topic = rospy.get_param(
            "~auth_overlay_topic",
            "/face_auth/overlay"
        )

        # Load known face
        rospy.loginfo(f"Loading known face from: {known_image_path}")
        if not os.path.exists(known_image_path):
            rospy.logerr(f"Image file not found: {known_image_path}")
            rospy.logerr("Please provide a valid image file with ~known_image_path parameter")
            sys.exit(1)

        known_image = face_recognition.load_image_file(known_image_path)
        encodings = face_recognition.face_encodings(known_image)

        if len(encodings) == 0:
            rospy.logerr(f"No face found in {known_image_path}")
            sys.exit(1)

        self.known_faces = [encodings[0]]
        rospy.loginfo("Known face loaded successfully")

        # Setup publishers
        self.auth_status_pub = rospy.Publisher(
            auth_status_topic,
            String,
            queue_size=10
        )
        self.auth_overlay_pub = rospy.Publisher(
            auth_overlay_topic,
            Image,
            queue_size=1
        )
        # Publish compressed version for web UI
        self.auth_overlay_compressed_pub = rospy.Publisher(
            auth_overlay_topic + "/compressed",
            CompressedImage,
            queue_size=1
        )

        # Determine if camera topic is compressed
        self.is_compressed = "compressed" in camera_topic.lower()
        
        # Subscribe to camera
        if self.is_compressed:
            rospy.Subscriber(
                camera_topic,
                CompressedImage,
                self.compressed_image_callback,
                queue_size=1,
                buff_size=2**24
            )
        else:
            rospy.Subscriber(
                camera_topic,
                Image,
                self.image_callback,
                queue_size=1,
                buff_size=2**24
            )

        rospy.loginfo(f"Subscribed to {camera_topic} ({'compressed' if self.is_compressed else 'raw'})")
        rospy.loginfo(f"Publishing auth status to {auth_status_topic}")
        rospy.loginfo(f"Publishing auth overlay to {auth_overlay_topic}")

    def compressed_image_callback(self, msg):
        """Handle compressed image messages"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                rospy.logerr("Failed to decode compressed image")
                return
            self.process_frame(frame, msg.header)
        except Exception as e:
            rospy.logerr(f"Error processing compressed image: {e}")

    def image_callback(self, msg):
        """Handle raw image messages"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_frame(frame, msg.header)
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_frame(self, frame, header):
        """Process a frame for face recognition"""
        # ----------------------------
        # Face recognition logic
        # ----------------------------
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        face_locations = face_recognition.face_locations(
            rgb_small_frame,
            model="hog"
        )

        face_encodings = face_recognition.face_encodings(
            rgb_small_frame,
            face_locations
        )

        overlay = frame.copy()
        any_authenticated = False
        any_unauthenticated = False

        for (top, right, bottom, left), face_encoding in zip(
            face_locations,
            face_encodings
        ):
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            matches = face_recognition.compare_faces(
                self.known_faces,
                face_encoding,
                tolerance=0.5
            )

            if True in matches:
                label = "Authenticated"
                color = (0, 255, 0)
                any_authenticated = True
            else:
                label = "Unauthenticated"
                color = (0, 0, 255)
                any_unauthenticated = True

            cv2.rectangle(overlay, (left, top), (right, bottom), color, 2)
            cv2.rectangle(
                overlay,
                (left, bottom - 28),
                (right, bottom),
                color,
                cv2.FILLED
            )
            cv2.putText(
                overlay,
                label,
                (left + 6, bottom - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        if any_unauthenticated:
            status = "unauthorized"
        elif any_authenticated:
            status = "authorized"
        else:
            status = "no_face"

        self.auth_status_pub.publish(String(data=status))

        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = header
            self.auth_overlay_pub.publish(overlay_msg)
            
            # Publish compressed version for web UI
            _, jpeg_data = cv2.imencode('.jpg', overlay)
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = jpeg_data.tobytes()
            self.auth_overlay_compressed_pub.publish(compressed_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

        # ----------------------------
        # Optional: display window
        # ----------------------------
        if self.display:
            cv2.imshow("ROS Face Authentication", overlay)
            cv2.waitKey(1)


# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    rospy.init_node("face_authentication_node")

    node = FaceAuthNode()

    rospy.loginfo("Face authentication node running")
    rospy.spin()

    cv2.destroyAllWindows()
