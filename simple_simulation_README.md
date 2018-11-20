Launching:
 
roslaunch uuv_descriptions herkules_ship_wreck.launch -> load map with ship
roslaunch mybot_gazebo mybot_world.launch -> spawn AUV and gates
roslaunch mybot_description mybot_rviz.launch -> launch RVIZ (and camera sensor)
roslaunch AUV.launch -> launch gate detection algorithm
rosrun simple_controller self_driving.py -> launch simple controller for navigation



Issues:

У аппарата перепутаны зад и перед (потом пофиксим)
Гравитацию нужно отключать в gazebo, нажав на аппарат и сняв галочку с поля gravity

How to control:

1) Using keys: roslaunch mybot_navigation mybot_teleop.launch
2) Self driving: rosrun simple_controller self_driving.py

Robot location topic: "/odometry/filtered"
Robot velocity topic: "/cmd_vel"

To control:
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom) -> узнаем положение
sub_g = rospy.Subscriber("/gate", Pose2D, callback) -> узнаем координаты центра ворот
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) -> перемещаемся 


