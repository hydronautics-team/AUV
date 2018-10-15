#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"


void gateCallback(const geometry_msgs::Pose2DConstPtr& msg)
{
    ROS_INFO("%f %f", (*msg).x, (*msg).y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locators_handler");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<geometry_msgs::Pose2D>("/gate", 100, gateCallback);

  ros::spin();

  return 0;
}
