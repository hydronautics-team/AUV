#include "MoveCenteringServer.h"

MoveCenteringServer::MoveCenteringServer(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory):
        MoveActionServerBase(actionName, velocityTopic, twistFactory) { };

void MoveCenteringServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}