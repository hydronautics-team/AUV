#include "simple_simulation/SimpleMoveCenteringServer.h"

SimpleMoveCenteringServer::SimpleMoveCenteringServer(const std::string& actionName) : SimpleMoveActionServerBase(actionName) { };

void SimpleMoveCenteringServer::executeCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}