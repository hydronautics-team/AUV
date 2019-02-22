#include <twist/RealTwistPublisher.h>
#include <auv_common/VelocityCmd.h>

RealTwistPublisher::RealTwistPublisher(ros::NodeHandle &nodeHandle, const std::string &velocityService) {
    velocityClient = nodeHandle.serviceClient<auv_common::VelocityCmd>(velocityService, true);
}

bool RealTwistPublisher::publishTwist(const geometry_msgs::Twist &twist) {
    auv_common::VelocityCmd cmd;
    cmd.request.twist = twist;
    if (!velocityClient.call(cmd))
    {
        return false;
    }
    return cmd.response.success.data;
}

