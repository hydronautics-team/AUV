#include "MoveByTimeServer.h"


MoveByTimeServer::MoveByTimeServer(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory):
        MoveActionServerBase(actionName, velocityTopic, twistFactory) { };

void MoveByTimeServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {

    /* TODO: Check responses */
    geometry_msgs::Twist startMsg = createTwistFromGoal(*goal);
    auv_common::VelocityCmd startCmd;
    startCmd.request.twist = startMsg;
    ros::service::call(velocityService, startCmd);

    if (goal->value != auv_common::MoveGoal::VALUE_INFINITY) {
        int timeMs = goal->value;
        float time = timeMs / 1000.0f;
        if (time != 0) {
            ros::Duration(time).sleep();
            geometry_msgs::Twist endMsg = twistFactory->createStopTwist();
            auv_common::VelocityCmd endCmd;
            endCmd.request.twist = endMsg;
            ros::service::call(velocityService, endCmd);

            ros::Duration(0.8).sleep();
        }
    } else if (goal->holdIfInfinityValue) {
        while (true)
        {
        }
    }

    actionServer.setSucceeded();
}