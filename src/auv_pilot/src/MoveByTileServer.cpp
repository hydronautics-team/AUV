#include "MoveByTileServer.h"

MoveByTileServer::MoveByTileServer(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory):
        MoveActionServerBase(actionName, velocityTopic, twistFactory) { };

void MoveByTileServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}