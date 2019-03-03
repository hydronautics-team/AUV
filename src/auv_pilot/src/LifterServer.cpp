#include <LifterServer.h>


LifterServer::LifterServer(const std::string &actionName, const std::string &lifterService) :
        lifterService(lifterService),
        actionServer(nodeHandle, actionName, boost::bind(&LifterServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void LifterServer::goalCallback(const auv_common::LifterGoalConstPtr &goal) {
    // Temporary stub
    actionServer.setSucceeded();
}