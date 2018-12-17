#include "MoveByTileServer.h"

MoveByTileServer::MoveByTileServer(const std::string &actionName, const std::string& velocityTopic):
    MoveActionServerBase(actionName, velocityTopic) { };

void MoveByTileServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}