#include "MoveByTimeServer.h"

MoveByTimeServer::MoveByTimeServer(const std::string& actionName, const std::string& velocityTopic):
    MoveActionServerBase(actionName, velocityTopic) { };

void MoveByTimeServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {

    geometry_msgs::Twist startMsg = createDirectionTwist(goal->direction);
    safePublish(startMsg);

    int time = goal->value;
    if (time != 0) {
        ros::Duration(time).sleep();
        geometry_msgs::Twist endMsg = createDirectionTwist(auv_common::MoveGoal::STOP);
        safePublish(endMsg);
    }

    actionServer.setSucceeded();
}