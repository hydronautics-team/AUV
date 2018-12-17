#include "MoveCenteringServer.h"

MoveCenteringServer::MoveCenteringServer(const std::string& actionName, const std::string& velocityTopic):
    MoveActionServerBase(actionName, velocityTopic) { };

void MoveCenteringServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}