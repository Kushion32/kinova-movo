# React UI for Movo Waypoint Navigation

This directory contains React components for controlling the Movo robot's navigation through a web interface.

## Setup Instructions

### 1. Install rosbridge_suite

On your robot or ROS machine:
```bash
sudo apt-get install ros-noetic-rosbridge-suite
```

### 2. Start rosbridge server

This creates a WebSocket connection that allows the browser to communicate with ROS:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

By default, this runs on port 9090.

### 3. Create a React project (if you don't have one)

```bash
npx create-react-app movo-nav-ui
cd movo-nav-ui
```

### 4. Install roslib

```bash
npm install roslib
```

### 5. Copy the components

Copy the following files into your React project's `src/` directory:
- `NavButton.jsx`
- `WaypointDashboard.jsx`

### 6. Update your App.js

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

### 7. Update waypoint names

Edit `WaypointDashboard.jsx` and replace the `locations` array with your actual waypoint names from `movo_waypoints.yaml`.

### 8. Start your React app

```bash
npm start
```

The UI will open at http://localhost:3000

## Usage

1. Make sure the robot and navigation system are running (see WORKSPACE_GUIDE.md)
2. Make sure rosbridge is running: `roslaunch rosbridge_server rosbridge_websocket.launch`
3. Start the waypoint navigator: `rosrun movo_nav goto_points.py`
4. Open the React UI in your browser
5. Click on waypoint buttons to send the robot to locations

## Architecture

- **React UI**: Sends waypoint names to `/ui_navigation_command` topic via WebSocket
- **rosbridge_suite**: Translates WebSocket messages to ROS messages
- **goto_points.py**: Subscribes to `/ui_navigation_command` and sends goals to move_base

## Troubleshooting

### Cannot connect to rosbridge
- Check that rosbridge is running: `rostopic list | grep rosbridge`
- Verify the WebSocket URL in `NavButton.jsx` matches your setup
- If running on a different machine, replace `localhost` with the robot's IP address

### Commands not working
- Verify the navigator is running: `rosnode list | grep waypoint`
- Check that waypoints exist: `rostopic echo /ui_navigation_command`
- Verify waypoint names match those in your YAML file

### CORS issues
If running the React app from a different machine:
1. Make sure your firewall allows port 9090
2. Update the rosbridge launch to allow external connections if needed
