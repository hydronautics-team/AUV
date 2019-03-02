# Autonomus Underwater Vehicle AI Source Code

For full information, see our [Wiki](https://github.com/hidronautics/AUV/wiki).

## Building and launching
Install dependencies:
```bash
sudo apt-get install ros-kinetic-rosbridge-server ros-kinetic-web-video-server ros-kinetic-image-view ros-kinetic-actionlib ros-kinetic-serial ros-kinetic-smach ros-kinetic-smach-viewer
```
Initialize and update git submodules used in project:
```bash
git submodule init
git submodule update
```
Use following commands to build and launch:
```bash
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch launch/AUV.launch
```
For more details see our wiki.
To build code with locally built OpenCV (not from ROS distribution), uncomment corresponding lines in [src/auv_vision/CMakeLists.txt](src/auv_vision/CMakeLists.txt).
