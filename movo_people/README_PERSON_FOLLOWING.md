# Person Following Navigation for MOVO Robot

## Overview

This system enables the MOVO robot to guide a person to a location while ensuring the person stays within 1-2 meters of the robot during navigation. If the person stops following, the robot will stop and turn around to face them.

## Features

- **Dual Detection System**:
  - **Camera-based detection**: Uses YOLO person detection to visually confirm person presence
  - **Lidar-based detection**: Uses laser scan data to detect person/legs within specified distance range

- **Automatic Behavior**:
  - Robot continuously monitors for person presence during navigation
  - If person stops following (not detected for > 3 seconds), robot stops immediately
  - Robot turns to face the person's last known location
  - Navigation resumes automatically when person is detected again
  - If person doesn't return within 30 seconds, navigation is aborted

## Installation

The code is already integrated into the MOVO workspace. No additional installation needed.

## Files Created/Modified

### New Files:
1. `/src/kinova-movo/movo_people/scripts/person_following_monitor.py`
   - Main monitoring node that tracks person presence
   - Processes laser scan and camera data
   - Publishes person following status

2. `/src/kinova-movo/movo_people/launch/person_following_navigation.launch`
   - Launch file to start person following system

### Modified Files:
1. `/src/kinova-movo/movo_common/movo_ros/src/movo/move_base.py`
   - Integrated person following checks into navigation loop
   - Added pause/resume behavior when person stops following
   - Publishes navigation status for monitoring node

## Usage

### Starting the System

1. **Start the person following monitor**:
```bash
roslaunch movo_people person_following_navigation.launch
```

2. **Send navigation goals as usual**:
   - Via RViz: Use "2D Nav Goal" tool
   - Via command line or code: Publish to `/move_base_simple/goal`

### Parameters

You can customize the behavior by adjusting parameters in the launch file:

- `enable_person_following` (default: true): Enable/disable person following monitoring
- `min_follow_distance` (default: 0.5m): Minimum distance person should be from robot
- `max_follow_distance` (default: 2.5m): Maximum distance person can be from robot
- `person_lost_timeout` (default: 3.0s): How long to wait before considering person lost
- `scan_angle_range` (default: 120°): Angular range to scan for person
- `min_obstacle_distance` (default: 0.3m): Minimum distance for obstacle detection

### Example: Custom Parameters

```bash
roslaunch movo_people person_following_navigation.launch \
  min_follow_distance:=1.0 \
  max_follow_distance:=2.0 \
  person_lost_timeout:=5.0
```

## How It Works

### Person Detection

1. **Lidar Detection**:
   - Scans laser data in a 240° arc behind and around the robot
   - Clusters nearby laser points that could represent human legs/body
   - Identifies clusters with appropriate width (0.05-0.8m) and point count
   - Calculates distance and angle to detected person

2. **Camera Detection**:
   - Uses YOLOv8 model to detect people in camera images
   - Confirms visual presence of person
   - Finds closest person in frame

3. **Combined Logic**:
   - Person is considered "following" if detected by either sensor
   - Last detection time is tracked
   - If no detection for > 3 seconds, person is considered "lost"

### Navigation Behavior

1. **Normal Navigation**:
   - Robot navigates to goal while monitoring person status
   - Person following status is published every 100ms

2. **Person Stops Following**:
   - Robot immediately stops movement
   - Publishes stop command to `/movo/teleop/cmd_vel`
   - Turns to face person's last known angle
   - Waits for person to return (up to 30 seconds)

3. **Person Returns**:
   - Robot detects person again
   - Logs "Person detected again - resuming navigation"
   - Continues toward navigation goal

4. **Person Doesn't Return**:
   - After 30 second timeout, navigation is aborted
   - Robot remains stopped
   - Error logged: "Person did not return - aborting navigation"

## Topics

### Published:
- `/movo/person_following_status` (std_msgs/Bool): True if person is following
- `/movo/navigation_active` (std_msgs/Bool): True when navigation goal is active
- `/movo/teleop/cmd_vel` (geometry_msgs/Twist): Direct velocity commands for stopping/turning

### Subscribed:
- `/scan` (sensor_msgs/LaserScan): Laser scan data for person detection
- `/camera/color/image_raw` (sensor_msgs/Image): Camera images for person detection
- `/movo/navigation_active` (std_msgs/Bool): Navigation status from move_base

## Customization

### Adjusting Detection Range

Edit [person_following_monitor.py](../scripts/person_following_monitor.py):

```python
# Line ~24-27
self.min_follow_distance = 1.0  # Change minimum distance
self.max_follow_distance = 2.0  # Change maximum distance
```

### Adjusting Turn Behavior

Edit the `search_for_person()` method (line ~280):

```python
turn_speed = 0.5  # Increase for faster turning
```

### Disabling Person Following

Launch move_base with person following disabled:

```bash
roslaunch movo_people person_following_navigation.launch enable_person_following:=false
```

Or set parameter directly:
```bash
rosparam set /movo_move_base/enable_person_following false
```

## Troubleshooting

### Person Not Detected
- Check camera is publishing: `rostopic echo /camera/color/image_raw`
- Check laser scan is publishing: `rostopic echo /scan`
- Verify YOLO model exists: `/src/kinova-movo/movo_people/scripts/yolov8n.pt`
- Check person is within detection range (0.5-2.5m by default)

### Robot Doesn't Stop When Person Leaves
- Check person following monitor is running: `rosnode list | grep person_following`
- Verify monitoring is enabled: `rosparam get /movo_move_base/enable_person_following`
- Check topic connection: `rostopic info /movo/person_following_status`

### Robot Stops Unexpectedly
- Person may be outside detection range (too close or too far)
- Adjust `min_follow_distance` and `max_follow_distance` parameters
- Check for obstacles blocking laser scan

## Architecture

```
┌─────────────────────────────────────┐
│   person_following_monitor.py      │
│                                     │
│  ┌──────────────┐ ┌─────────────┐ │
│  │ Laser Scan   │ │   Camera    │ │
│  │  Detection   │ │  Detection  │ │
│  └──────┬───────┘ └──────┬──────┘ │
│         │                 │        │
│         └────────┬────────┘        │
│                  │                 │
│         ┌────────▼────────┐        │
│         │  Person Status  │        │
│         │   Publisher     │        │
│         └────────┬────────┘        │
└──────────────────┼─────────────────┘
                   │
         /movo/person_following_status
                   │
                   ▼
┌─────────────────────────────────────┐
│        move_base.py                 │
│                                     │
│  ┌──────────────────────────────┐  │
│  │  Navigation Loop             │  │
│  │  - Check person status       │  │
│  │  - Pause if person lost      │  │
│  │  - Resume when person returns│  │
│  └──────────────────────────────┘  │
└─────────────────────────────────────┘
```

## Safety Considerations

- Robot will stop if person is not detected
- 30-second timeout prevents indefinite waiting
- Emergency stop available via `/movo/teleop/abort_navigation`
- Always supervise the robot during operation

## Future Enhancements

Potential improvements:
- Voice announcements when person is lost
- Visual indicators (LED colors) for following status
- Multiple person tracking
- Person re-identification after occlusion
- Adaptive speed based on person distance
- Integration with social navigation behaviors
