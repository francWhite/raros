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
4. run led controller
    ```bash
   ros2 run raros led_controller
    ```
5. run micro-ros-agent
    ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   #or
   docker run -it --rm --privileged -v /dev:/dev --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0
    ```
6. connect arduino