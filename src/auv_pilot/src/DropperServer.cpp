#include <DropperServer.h>


DropperServer::DropperServer(const std::string &actionName, const std::string &dropperService) :
        dropperService(dropperService),
        actionServer(nodeHandle, actionName, boost::bind(&DropperServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void DropperServer::goalCallback(const auv_common::DropperGoalConstPtr &goal) {
    // Temporary stub
    actionServer.setSucceeded();
}