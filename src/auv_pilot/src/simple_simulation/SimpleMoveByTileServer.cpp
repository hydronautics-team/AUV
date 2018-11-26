#include "simple_simulation/SimpleMoveByTileServer.h"
#include <geometry_msgs/Twist.h>

SimpleMoveByTileServer::SimpleMoveByTileServer(const std::string& actionName):
        MoveActionServerBase(actionName),
        velocityPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1)) { };

void SimpleMoveByTileServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}