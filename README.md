# Autonomus Underwater Vehicle AI Source Code

## Building and launching
Install dependencies:
```bash
sudo apt-get install ros-kinetic-uvc-camera
```
Use following commands to build and launch:
```bash
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch AUV.launch
```
For more details see our wiki.
To build code with locally built OpenCV (not from ROS distribution), uncomment corresponding lines in [src/auv_vision/CMakeLists.txt](src/auv_vision/CMakeLists.txt).