# Workspace Structure and Commands Guide

This guide summarizes the workspace structure, where to find common functionality, and the frequently used commands plus the files/arguments they accept.

## Workspace layout (top level)

- `build/`: Catkin build artifacts (generated).
- `devel/`: Catkin devel space (generated).
- `logs/`: Build/test logs (generated).
- `src/`: Source tree. The main repository is `src/kinova-movo`.

## Repository layout (src/kinova-movo)

Key packages and where their functionality lives:

- `movo_bringup/`: Robot bring-up launch files (startup of robot components).
- `movo_description/`: URDF/Xacro and robot model assets.
- `movo_demos/`: Demo launch files, goals, configs, and maps.
  - `movo_demos/maps/`: Map YAML/PGM files used by navigation.
  - `movo_demos/launch/`: Demo launch files (e.g., navigation demos).
- `movo_gazebo/`: Gazebo simulation launch and worlds.
- `movo_moveit_config/` and `movo_7dof_moveit_config/`: MoveIt configs, demos, and scripts.
- `movo_nav/`: Navigation scripts and waypoint files.
  - `movo_nav/scripts/goto_points.py`: Waypoint navigation script.
  - `movo_nav/movo_waypoints.yaml`: Default waypoint definitions.
- `movo_msgs/`: Custom ROS messages.
- `movo_viz/`: RViz visualization launch files.
- `movo_robot/`: Core robot packages, drivers, and hardware interface.
- `movo_simulation/`: Simulation helpers and launch files.

## Functionality quick map

- **Bring up the robot**: `movo_bringup/launch/`
- **Navigation demos**: `movo_demos/launch/` and `movo_nav/scripts/`
- **Maps**: `movo_demos/maps/`
- **Waypoint navigation**: `movo_nav/scripts/goto_points.py` with `movo_nav/movo_waypoints.yaml`
- **MoveIt planning**: `movo_7dof_moveit_config/` and `movo_moveit_config/`
- **Visualization**: `movo_viz/launch/`
- **Gazebo simulation**: `movo_gazebo/launch/`

## Step-by-step: Running waypoint navigation

To use the waypoint navigation system, follow these steps in order:

### 1. Start the robot
First, bring up the robot hardware (or skip if using simulation):
```bash
# On movo2:
roslaunch movo_bringup movo2.launch

# On movo1:
roslaunch movo_bringup movo1.launch
```

### 2. Launch the navigation system
Start the map-based navigation stack:
```bash
roslaunch movo_demos map_nav.launch sim:=false local:=true map_file:=movo_map
```
Or alternatively:
```bash
roslaunch movo_demos robot_map_nav.launch map_file:=movo_map
```

### 3. Open RViz for visualization
In a new terminal:
```bash
roslaunch movo_viz view_robot.launch function:=map_nav
```

### 4. Set the initial pose in RViz
**IMPORTANT**: This step is required before the waypoint navigation will work!
- In RViz, click the "2D Pose Estimate" button in the top toolbar
- Click on the map where the robot is currently located
- Drag to set the robot's orientation
- The robot should now appear correctly positioned on the map

### 5. Run the waypoint navigation script
In a new terminal:
```bash
rosrun movo_nav goto_points.py _waypoints_file:=/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml
```

Once connected, you can:
- Type a waypoint name to navigate there
- Type `list` to see available waypoints
- Type `exit` to quit

## Step-by-step: Setting up UI-based waypoint navigation

To control the robot from a web browser interface:

### 1. Install Node.js and npm (one-time setup)
```bash
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### 2. Create the React app (one-time setup)
```bash
cd ~
npx create-react-app movo-nav-ui
cd movo-nav-ui
npm install roslib
```

### 3. Copy the UI components (one-time setup)
```bash
cp ~/catkin_ws/src/kinova-movo/movo_nav/ui/NavButton.jsx ~/movo-nav-ui/src/
cp ~/catkin_ws/src/kinova-movo/movo_nav/ui/WaypointDashboard.jsx ~/movo-nav-ui/src/
```

### 4. Update App.js (one-time setup)
Edit `~/movo-nav-ui/src/App.js`:
```javascript
import React from 'react';
import './App.css';
import WaypointDashboard from './WaypointDashboard';

function App() {
  return (
    <div className="App">
      <WaypointDashboard />
    </div>
  );
}

export default App;
```

### 5. Configure the rosbridge URL (one-time setup)
Edit `~/movo-nav-ui/src/NavButton.jsx` and update the WebSocket URL (line 10):
- For local testing: `url: 'ws://localhost:9090'`
- For remote connection to robot: `url: 'ws://movo2:9090'` (or robot's IP)

### 6. Update waypoint list (one-time setup)
Edit `~/movo-nav-ui/src/WaypointDashboard.jsx` and update the `locations` array with your actual waypoint names from `movo_waypoints.yaml`.

### 7. Install rosbridge (one-time setup)
```bash
sudo apt-get install ros-noetic-rosbridge-suite
```

### Running the UI navigation system:

**Terminal 1 - Start rosbridge:**
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

**Terminal 2 - Start navigation (follow steps 1-4 from "Running waypoint navigation" above)**

**Terminal 3 - Start waypoint navigator:**
```bash
rosrun movo_nav goto_points.py
```

**Terminal 4 - Start React UI:**
```bash
cd ~/movo-nav-ui
npm start
```

The UI will open at http://localhost:3000. Click buttons to send the robot to waypoints!

**Troubleshooting:**
- Check browser console (F12) for connection errors
- Verify rosbridge is running: `rosnode list | grep rosbridge`
- Test manually: `rostopic pub /ui_navigation_command std_msgs/String "data: 'waypoint_name'" -1`
- Monitor topic: `rostopic echo /ui_navigation_command`

## Commands and their inputs

The following commands are commonly used and include the key files/arguments they accept.

| Command | Purpose | Key file/arg inputs |
| --- | --- | --- |
| `roslaunch movo_demos map_nav.launch sim:=false local:=true map_file:=movo_map` | Start map navigation demo | `map_file`: map base name (expects matching files in `movo_demos/maps/`, e.g., `movo_map.yaml`/`movo_map.pgm`) |
| `roslaunch movo_viz view_robot.launch function:=map_nav` | RViz visualization for navigation | `function`: visualization mode (e.g., `map_nav`) |
| `roslaunch movo_demos robot_map_nav.launch map_file:=movo_map` | Navigation on robot with a map | `map_file`: map base name (in `movo_demos/maps/`) |
| `rosrun map_server map_saver -f ~/movo_map` | Save a map | `-f`: output map base name (writes `*.yaml` and `*.pgm`) |
| `rosrun gmapping slam_gmapping scan:=/movo/scan_multi _base_frame:=base_link _odom_frame:=odom _map_update_interval:=1.0` | Online SLAM | `scan`, `_base_frame`, `_odom_frame`, `_map_update_interval` |
| `rosrun joy joy_node` | Joystick driver | No file inputs |
| `rosrun map_server map_server /home/robohub/catkin_ws/src/kinova-movo/movo_demos/maps/floor_plan.yaml _frame_id:=map _topic:=/floor_plan` | Publish a static map | Map file path (`.yaml`) |
| `rosrun movo_nav goto_points.py _waypoints_file:=/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml _target:=storage_closet` | Navigate to a named waypoint | `_waypoints_file`: YAML file with named waypoints; `_target`: key in that YAML |
| `rostopic echo /clicked_point` | Inspect clicked point in RViz | No file inputs |
| `python3 realsense_person_counter.py` | Person counting script | Script location not in repo (run from wherever the file lives) |
| `rostopic echo /people/count` | Inspect people count topic | No file inputs |
| `ROS_NAMESPACE=/movo rosrun teleop_twist_keyboard teleop_twist_keyboard.py` | Keyboard teleop | No file inputs |

## Related references

- General setup and robot workflow: `README.md`
- Existing command list: `COMMANDS.md`
