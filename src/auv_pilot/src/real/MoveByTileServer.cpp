#include "real/MoveByTileServer.h"

MoveByTileServer::MoveByTileServer(const std::string &actionName) : MoveActionServerBase(actionName) { };

void MoveByTileServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}