import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const NavButton = ({ waypointName, ros }) => {
  const [navTopic, setNavTopic] = useState(null);

  useEffect(() => {
    if (!ros) return;

    // Setup the Topic
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/ui_navigation_command',
      messageType: 'std_msgs/String'
    });

    setNavTopic(topic);
  }, [ros]);

  const handleNavigate = () => {
    if (navTopic) {
      const msg = new ROSLIB.Message({
        data: waypointName
      });
      navTopic.publish(msg);
      console.log(`Command sent: ${waypointName}`);
    }
  };

  return (
    <button 
      onClick={handleNavigate}
      style={{ 
        padding: '10px 20px', 
        margin: '5px', 
        cursor: 'pointer',
        fontSize: '16px',
        borderRadius: '5px',
        backgroundColor: '#4CAF50',
        color: 'white',
        border: 'none'
      }}
    >
      Go to {waypointName}
    </button>
  );
};

export default NavButton;
