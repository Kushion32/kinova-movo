# Face Authentication ROS Node

This node detects faces in a camera feed and identifies whether they are authorized (matching a known face) or unauthorized.

## Setup

### 1. Install dependencies

```bash
pip3 install face_recognition
sudo apt-get install ros-noetic-cv-bridge
```

### 2. Prepare your known face image

Take a clear photo of the authorized person and save it. For example:
```bash
# Place your photo in the movo_nav package
mkdir -p ~/catkin_ws/src/kinova-movo/movo_nav/config
cp /path/to/your/photo.jpg ~/catkin_ws/src/kinova-movo/movo_nav/config/me.jpg
```

The image should:
- Contain exactly one face
- Be well-lit and clear
- Be a front-facing photo

## Running the Node

### Basic usage:
```bash
roslaunch movo_nav face_authentication.launch
```

### With custom camera topic:
```bash
roslaunch movo_nav face_authentication.launch camera_topic:=/kinect2/qhd/image_color_rect
```

### With custom known face image:
```bash
roslaunch movo_nav face_authentication.launch known_image_path:=/path/to/your/photo.jpg
```

### Without display window:
```bash
roslaunch movo_nav face_authentication.launch display:=false
```

## Topics

### Subscribed Topics:
- `/camera/image_raw` (sensor_msgs/Image): Camera feed (configurable)

### Published Topics:
- `/face_auth/status` (std_msgs/String): Authentication status
  - `"authorized"`: Recognized face detected
  - `"unauthorized"`: Unrecognized face detected
  - `"no_face"`: No face detected
  
- `/face_auth/overlay` (sensor_msgs/Image): Camera feed with bounding boxes
  - Green box: Authorized person
  - Red box: Unauthorized person

## Example: Monitor authentication status

```bash
rostopic echo /face_auth/status
```

## Example: View annotated video feed

```bash
rosrun image_view image_view image:=/face_auth/overlay
```

## Integration Example

Monitor for unauthorized people and trigger an alert:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def auth_callback(msg):
    if msg.data == "unauthorized":
        rospy.logwarn("UNAUTHORIZED PERSON DETECTED!")
        # Add your alert logic here (sound alarm, send notification, etc.)

rospy.init_node('security_monitor')
rospy.Subscriber('/face_auth/status', String, auth_callback)
rospy.spin()
```

## Troubleshooting

**No face found in image:**
- Make sure the image contains a clear, front-facing face
- Try a different photo with better lighting

**Poor detection accuracy:**
- Adjust the `tolerance` parameter in the code (line 115)
- Lower values = stricter matching (default: 0.5)

**Slow performance:**
- The node processes every frame; consider reducing camera frame rate
- Reduce image resolution in camera settings
