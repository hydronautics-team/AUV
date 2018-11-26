#include "real/MoveByTimeServer.h"

MoveByTimeServer::MoveByTimeServer(const std::string& actionName) : MoveActionServerBase(actionName) { };

void MoveByTimeServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}