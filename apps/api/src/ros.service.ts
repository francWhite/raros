import { ActionClient, Goal, Message, Ros, Service, ServiceRequest, Topic } from 'roslib';
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

    service.callService(new ServiceRequest(requestData), callback, errorCallback);
  }

  sendGoal(serverName: string, actionName: string, goalMessage: unknown, statusCallback: (status: unknown) => void) {
    const actionClient = new ActionClient({
      ros: this.ros,
      serverName: serverName,
      actionName: actionName,
      timeout: 10,
      omitStatus: true,
      omitResult: false,
      omitFeedback: false,
    });

    const goal = new Goal({
      actionClient: actionClient,
      goalMessage: goalMessage,
    });

    goal.on('status', statusCallback);
    goal.send();
  }

  readSingleMessageFromTopic<T>(topicName: string, messageType: string, callback: (message: T) => void) {
    const topic = new Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    topic.subscribe((message) => {
      callback(message as T);
      topic.unsubscribe();
    });
  }
}

export const rosService = new RosService();
