#include "simple_simulation/SimpleMoveByTimeServer.h"
#include <geometry_msgs/Twist.h>


SimpleMoveByTimeServer::SimpleMoveByTimeServer(const std::string& actionName):
        SimpleMoveActionServerBase(actionName) { };

void SimpleMoveByTimeServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {

    geometry_msgs::Twist startMsg = createDirectionTwist(goal->direction);
    publishToGazebo(startMsg);

    int time = goal->value;
    if (time != 0) {
        ros::Duration(time).sleep();
        geometry_msgs::Twist endMsg = createDirectionTwist(auv_common::MoveGoal::STOP);
        publishToGazebo(endMsg);
    }

    actionServer.setSucceeded();
}