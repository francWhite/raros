import ROSLIB from 'roslib';
import process from 'process';

const ros = new ROSLIB.Ros({
  url: process.env.ROS_URL || 'ws://localhost:9090',
});

ros.on('connection', () => {
  console.log('connected to websocket server.');
});

ros.on('error', (error) => {
  console.log('error connecting to websocket server: ', error);
});

ros.on('close', () => {
  console.log('connection to websocket server closed.');
});

export function publishToTopic(name: string, message: string) {
  const topic = new ROSLIB.Topic({
    ros,
    name: name,
    messageType: 'std_msgs/String',
  });

  const msg = new ROSLIB.Message({
    data: message,
  });

  topic.publish(msg);
}
