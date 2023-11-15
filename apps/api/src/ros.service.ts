import { Message, Ros, Service, ServiceRequest, Topic } from 'roslib';
import process from 'process';

class RosService {
  private readonly ros: Ros;

  constructor() {
    this.ros = new Ros({});

    this.ros.on('connection', () => {
      console.log('connected to websocket server.');
    });

    this.ros.on('error', () => {
      console.log('error connecting to websocket server');
      this.ros.close();
    });

    this.ros.on('close', () => {
      console.log('connection to websocket server closed. Reconnect will be attempted in 1 second.');
      setTimeout(() => {
        this.connect();
      }, 1000);
    });
  }

  connect() {
    const url = process.env.ROS_URL || 'ws://localhost:9090';
    this.ros.connect(url);
  }

  publishToTopic(name: string, message: string) {
    const topic = new Topic({
      ros: this.ros,
      name: name,
      messageType: 'std_msgs/String',
    });

    const msg = new Message({
      data: message,
    });

    topic.publish(msg);
  }

  callService(name: string, serviceType: string, payload: unknown, callback: (result: unknown) => void) {
    const service = new Service({
      ros: this.ros,
      name: name,
      serviceType: serviceType,
    });

    service.callService(new ServiceRequest(payload), callback);
  }
}

export const rosService = new RosService();
