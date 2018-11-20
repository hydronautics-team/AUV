#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from geometry_msgs.msg import Pose2D
 
x = 0.0
y = 0.0 
theta = 0.0

x_gate = 0.0
y_gate = 0.0

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.x)

    global x_gate
    global y_gate

    x_gate = msg.x
    y_gate = msg.y
 
def newOdom(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
 
rospy.init_node("speed_controller")
 
sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
sub_g = rospy.Subscriber("/gate", Pose2D, callback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
speed = Twist()
 
r = rospy.Rate(4)

 
while not rospy.is_shutdown():
 
    if x_gate > 400:
        speed.linear.x = -0.5
        speed.angular.z = -0.3

    else:
        if x_gate < 400:
            speed.linear.x = -0.5
            speed.angular.z = 0.3

 
    pub.publish(speed)
    r.sleep()  

if __name__ == '__main__':
    try:
        self_driving()
    except rospy.ROSInterruptException:
        pass  
