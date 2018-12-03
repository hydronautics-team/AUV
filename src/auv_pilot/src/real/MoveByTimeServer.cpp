#include "real/MoveByTimeServer.h"

MoveByTimeServer::MoveByTimeServer(const std::string& actionName) : MoveActionServerBase(actionName) { };

void MoveByTimeServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}