#include "simple_simulation/SimpleMoveByTileServer.h"
#include <geometry_msgs/Twist.h>

SimpleMoveByTileServer::SimpleMoveByTileServer(const std::string& actionName):
        SimpleMoveActionServerBase(actionName) { };

void SimpleMoveByTileServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}