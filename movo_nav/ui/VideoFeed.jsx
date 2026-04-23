import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const VideoFeed = ({ ros, topicName = '/kinect2/qhd/image_color_rect/compressed' }) => {
  const [imageSrc, setImageSrc] = useState(null);

  useEffect(() => {
    if (!ros) return;

    // Subscribe to compressed image topic
    const imageListener = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: 'sensor_msgs/CompressedImage'
    });

    imageListener.subscribe((message) => {
      // Convert base64 image data to displayable format
      const imageData = `data:image/jpeg;base64,${message.data}`;
      setImageSrc(imageData);
    });

    return () => {
      imageListener.unsubscribe();
    };
  }, [ros, topicName]);

  return (
    <div style={{
      border: '2px solid #ddd',
      borderRadius: '8px',
      padding: '10px',
      backgroundColor: '#000',
      maxWidth: '640px',
      margin: '10px 0'
    }}>
      <h3 style={{ color: '#fff', margin: '0 0 10px 0' }}>Camera Feed</h3>
      {imageSrc ? (
        <img 
          src={imageSrc} 
          alt="Robot camera feed" 
          style={{ 
            width: '100%', 
            height: 'auto',
            borderRadius: '4px'
          }}
        />
      ) : (
        <div style={{ 
          color: '#999', 
          padding: '40px', 
          textAlign: 'center' 
        }}>
          Waiting for camera feed...
        </div>
      )}
      <p style={{ 
        color: '#999', 
        fontSize: '12px', 
        margin: '5px 0 0 0' 
      }}>
        Topic: {topicName}
      </p>
    </div>
  );
};

export default VideoFeed;
