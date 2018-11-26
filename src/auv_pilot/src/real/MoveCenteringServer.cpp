#include "real/MoveCenteringServer.h"

MoveCenteringServer::MoveCenteringServer(const std::string& actionName) : MoveActionServerBase(actionName) { };

void MoveCenteringServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}