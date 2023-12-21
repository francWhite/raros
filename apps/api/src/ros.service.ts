import { Ros, Service, ServiceRequest, Topic } from 'roslib';
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

  callService<TResponse>(
    name: string,
    serviceType: string,
    requestData: unknown,
    callback: (result: TResponse) => void,
    errorCallback?: (error: string) => void,
  ) {
    const service = new Service({
      ros: this.ros,
      name: name,
      serviceType: serviceType,
    });

    const timeout = setTimeout(() => {
      errorCallback && errorCallback('no response received within 5 second');
    }, 5000);

    service.callService(new ServiceRequest(requestData), callback, errorCallback);
    clearTimeout(timeout);
  }

  readSingleMessageFromTopic<T>(
    topicName: string,
    messageType: string,
    callback: (message: T) => void,
    errorCallback?: (error: string) => void,
  ) {
    const topic = new Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    const timeout = setTimeout(() => {
      topic.unsubscribe();
      errorCallback && errorCallback('no message received within 5 second');
    }, 5000);

    topic.subscribe((message) => {
      clearTimeout(timeout);
      callback(message as T);
      topic.unsubscribe();
    });
  }
}

export const rosService = new RosService();
