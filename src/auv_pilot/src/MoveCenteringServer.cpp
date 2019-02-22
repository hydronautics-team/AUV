#include "MoveCenteringServer.h"

MoveCenteringServer::MoveCenteringServer(const std::string& actionName, bool isSimulation, const std::string& velocityTopicOrService,
                                         const TwistFactory& twistFactory):
    MoveActionServerBase(actionName, isSimulation, velocityTopicOrService, twistFactory) { };

void MoveCenteringServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {
    /** Temporary stub */
    actionServer.setAborted();
}