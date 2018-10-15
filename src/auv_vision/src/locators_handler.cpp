#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/gate/GateDescriptor.h"
#include "../include/msg/gate_msg.h"

void chatterCallback(const auv_vision::gate_msgConstPtr& msg)
{
  GateDescriptor gate = GateDescriptor::fromMsg(*msg);
  if (gate.hasGate())
    ROS_INFO("%s", "Gates detected");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locators_handler");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<auv_vision::gate_msg>("/gate", 100, chatterCallback);

  ros::spin();

  return 0;
}
