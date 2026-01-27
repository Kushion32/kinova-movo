#!/usr/bin/env python

# This script performs real-time person detection using YOLOv8 and a RealSense camera.
# It uses the pyrealsense2 SDK directly, without relying on ROS.

import cv2
import numpy as np
import pyrealsense2 as rs # Direct RealSense SDK library
import time
from ultralytics import YOLO

# --- CONFIGURATION (Updated) ---
# Time delay to wait for auto-exposure/gain to settle after stream start
STARTUP_DELAY_SEC = 1.0 
# New: Timeout in milliseconds for waiting for a new frame set. 
# Increased from default 5000ms to 15000ms (15 seconds) to prevent timeout errors 
# during high CPU load or initial connection.
FRAME_TIMEOUT_MS = 60000 
# ----------------------------------------------------

# --- YOLO Detector Class (Unchanged, as it's independent of ROS/RealSense) ---

class YoloPersonDetector:
    """
    Wraps YOLOv8 inference and draws person detections on BGR images.
    Note: This requires the 'ultralytics' library to be installed.
    """

    def __init__(self, model_path="yolov8n.pt", conf=0.25):
        self.model = YOLO(model_path)
        self.conf = conf
        print(f"YOLOv8 Model loaded: {model_path} with confidence threshold {conf}")

    def detect_people(self, image):
        """
        Runs YOLOv8 on the provided BGR image.
        :param image: OpenCV BGR image (numpy array).
        :returns: (count, annotated_image, list_of_boxes)
        """
        # Run prediction, limiting output to the first result [0]
        results = self.model.predict(source=image, conf=self.conf, verbose=False)[0]

        annotated = image.copy()
        person_boxes = []
        
        for box in results.boxes:
            cls = int(box.cls[0])
            if cls == 0:  # person class in COCO
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                person_boxes.append((x1, y1, x2, y2))
                
                # Draw bounding box and label
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    "person",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        return len(person_boxes), annotated, person_boxes


# --- RealSense Standalone Stream Handler ---

class RealSenseDetectorApp:
    """
    Handles the RealSense pipeline, synchronizes frames, runs detection, and visualizes results.
    """
    def __init__(self, model_path="yolov8n.pt", conf=0.25):
        
        # 1. Initialize YOLO Detector
        self.detector = YoloPersonDetector(model_path=model_path, conf=conf)

        # 2. Initialize RealSense Pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color) # Alignment object to align depth to color stream

        # 3. Enable Streams (adjust resolution/FPS as needed)
        # Using 640x480 for color and depth to reduce computational load
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        print("Starting RealSense stream...")
        self.profile = self.pipeline.start(self.config)
        
        # 4. Wait for auto-exposure to settle (optional but recommended)
        time.sleep(STARTUP_DELAY_SEC)
        print("RealSense stream started. Press 'q' to exit.")


    def process_frames(self, color_image, depth_image):
        """
        Runs the YOLO detection on the color image and prints the count and depth.
        """
        try:
            # 1. Run YOLO detection
            person_count, annotated_img, boxes = self.detector.detect_people(color_image)

            # 2. Estimate depth at the center of each detected box
            if boxes and depth_image is not None:
                depth_values = []
                height, width = depth_image.shape[:2]
                
                for (x1, y1, x2, y2) in boxes:
                    # Calculate center point
                    cx = max(0, min(width - 1, (x1 + x2) // 2))
                    cy = max(0, min(height - 1, (y1 + y2) // 2))
                    
                    # Depth value is in millimeters (z16 format)
                    depth_mm = depth_image[cy, cx]
                    depth_values.append(depth_mm)
                    
                    # Add depth label to the visualization
                    if depth_mm > 0:
                        cv2.putText(annotated_img, f"{depth_mm/1000.0:.2f}m", (x1 + 5, y2 - 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                print(f"[{time.strftime('%H:%M:%S')}] Detected {person_count} person(s). Distances (mm): {depth_values}")

            # 3. Add person count to the top left corner of the visualization
            cv2.putText(annotated_img, f"Count: {person_count}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # 4. Visualize detections
            cv2.imshow("People Detection (Press 'q' to close)", annotated_img)
            
            return person_count
            
        except Exception as e:
            print(f"Error in detection logic: {e}")
            return 0


    def run(self):
        """
        The main loop for capturing and processing frames.
        """
        try:
            while True:
                # 1. Wait for a coherent frameset of depth and color, using the increased timeout
                frames = self.pipeline.wait_for_frames(FRAME_TIMEOUT_MS)

                # 2. Align the depth frame to the color frame
                aligned_frames = self.align.process(frames)
                
                # 3. Get aligned frames
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # 4. Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # 5. Run processing and detection
                self.process_frames(color_image, depth_image)

                # Exit on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Application interrupted by user.")
        # Catch the specific RealSense timeout error explicitly here
        except RuntimeError as e:
            # Check if the error message contains the specific timeout phrase
            if "Frame didn't arrive within" in str(e):
                 print(f"Failed to run RealSense Detector App: RealSense frame timeout exceeded ({FRAME_TIMEOUT_MS}ms).")
                 print("This usually means the camera connection is unstable or your system is under heavy load. Please try another USB 3.0 port.")
            else:
                 print(f"Failed to run RealSense Detector App: {e}")
            
        finally:
            # Stop streaming and clean up
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    # Ensure you have the 'ultralytics' and 'pyrealsense2' packages installed.
    try:
        app = RealSenseDetectorApp()
        app.run()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        print("Final check: Is 'pyrealsense2' installed and is the camera connected?")