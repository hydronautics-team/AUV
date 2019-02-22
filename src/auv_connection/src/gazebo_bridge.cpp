#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include <auv_common/VelocityCmd.h>

static const std::string GAZEBO_VELOCITY_TOPIC = "/cmd_vel";

static const std::string VELOCITY_SERVICE = "velocity_service";

ros::Publisher velocityPublisher;

bool movementCallback(auv_common::VelocityCmd::Request& velocityRequest,
                       auv_common::VelocityCmd::Response& velocityResponse);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_bridge");
    ros::NodeHandle nodeHandle;

    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(GAZEBO_VELOCITY_TOPIC, 100);

    ros::ServiceServer velocity_srv = nodeHandle.advertiseService(VELOCITY_SERVICE, movementCallback);

    ros::spin();

    return 0;
}


bool movementCallback(auv_common::VelocityCmd::Request& velocityRequest,
                       auv_common::VelocityCmd::Response& velocityResponse) {
    ros::Rate pollRate(100);
    while (velocityPublisher.getNumSubscribers() == 0)
        pollRate.sleep();
    velocityPublisher.publish(velocityRequest.twist);

    velocityResponse.success.data = true;

    return true;
}