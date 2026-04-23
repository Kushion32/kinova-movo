import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import NavButton from './NavButton';
import VideoFeed from './VideoFeed';

const WaypointDashboard = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  // Update this list with your actual waypoint names from movo_waypoints.yaml
  const locations = ['kitchen', 'lab', 'charging_dock', 'entrance'];

  useEffect(() => {
    // Initialize ROS Connection (shared across components)
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Update this to match your rosbridge URL
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS websocket server.');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.log('Error connecting to ROS websocket server: ', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS websocket server closed.');
      setConnected(false);
    });

    setRos(rosInstance);

    return () => rosInstance.close();
  }, []);

  return (
    <div className="dashboard" style={{
      padding: '20px',
      fontFamily: 'Arial, sans-serif'
    }}>
      <h2>Movo Navigation Control</h2>
      
      {/* Connection status */}
      <div style={{
        padding: '10px',
        marginBottom: '15px',
        borderRadius: '5px',
        backgroundColor: connected ? '#d4edda' : '#f8d7da',
        color: connected ? '#155724' : '#721c24',
        border: `1px solid ${connected ? '#c3e6cb' : '#f5c6cb'}`
      }}>
        {connected ? '✓ Connected to ROS' : '✗ Disconnected from ROS'}
      </div>

      <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap' }}>
        {/* Navigation controls */}
        <div>
          <h3>Navigation</h3>
          <p>Click a button to send the robot to that location</p>
          <div style={{ display: 'flex', flexDirection: 'column', maxWidth: '300px' }}>
            {locations.map(loc => (
              <NavButton key={loc} waypointName={loc} ros={ros} />
            ))}
          </div>
        </div>

        {/* Video feed */}
        <div>
          <VideoFeed ros={ros} topicName="/kinect2/qhd/image_color_rect/compressed" />
        </div>
      </div>
    </div>
  );
};

export default WaypointDashboard;
