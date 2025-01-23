# Safety_system_ros2_ws
Collection of packages used to test various sensors for the development of a safety system for an amr tractor

The ROS2 nodes are dependent on the Bosch off highway ROS2 packages found on github.
Also the A YOLO ROS2 package from github is used to apply inference results to the video streaming node.


**Braking control node**
This node adds braking markers in Rviz2 based on the x and y positions of detected objects from the radar and ultrasonic sensors.

It also adds markers displaying the distance, velocity and exist probability from the sensor/msg topic from the Bosch radar node.


**Vehicle simulator**
This node is used to provide the radar sensor with vehicle speed and yaw rate date over can-bus


**video publisher**
A node that creates a simple video publisher of raw camera data together with a camera/info topic so that the camera image can be viewed in Rviz2
