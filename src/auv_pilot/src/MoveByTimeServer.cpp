#include "MoveByTimeServer.h"
#include <iostream>

MoveByTimeServer::MoveByTimeServer(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory):
    MoveActionServerBase(actionName, velocityTopic, twistFactory) { };

void MoveByTimeServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {

    geometry_msgs::Twist startMsg = twistFactory->createDirectionTwist(goal->direction);
    safePublish(startMsg);

    int timeMs = goal->value;
    float time = timeMs / 1000.0f;
    if (time != 0) {
        ros::Duration(time).sleep();
        geometry_msgs::Twist endMsg = twistFactory->createDirectionTwist(auv_common::MoveGoal::STOP);
        safePublish(endMsg);
        ros::Duration(2.0).sleep();
    }

    actionServer.setSucceeded();
}