# Robot controller in ROS2

## Demo
___

1. clone repository
2. build workspace
    ```bash
   colcon build
    ```
3. source workspace
    ```bash
   source install/setup.bash
    ```
4. run micro-ros-agent
    ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   #or
   docker run -it --rm --privileged -v /dev:/dev --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0
    ```
5. connect arduino
6. run led controller
    ```bash
   ros2 run raros led_controller
    ```

## with API
1. run ros_bridge
    ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ```
2. run led endpoint
    ```bash
   ros2 run raros led_api_endpoint
    ```
3. switch to api directory
4. install packages
    ```bash
   npm install
    ```
5. run api
    ```bash
    npm run start build && npm run start
      ```
6. make call with postman / curl
    ```bash
    curl -X POST http://192.168.0.54:8000/api/controller/led -H "Content-Type: application/json" -d '{"state": "on"}'
    ```