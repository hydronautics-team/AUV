#include "simple_simulation/SimpleMoveByTimeServer.h"
#include <geometry_msgs/Twist.h>

SimpleMoveByTimeServer::SimpleMoveByTimeServer(const std::string& actionName):
        MoveActionServerBase(actionName),
        velocityPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1)) { };

void SimpleMoveByTimeServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}