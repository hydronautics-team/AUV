#include <DropperServer.h>
#include <auv_common/DropperCmd.h>


DropperServer::DropperServer(const std::string &actionName, const std::string &dropperService) :
        dropperService(dropperService),
        actionServer(nodeHandle, actionName, boost::bind(&DropperServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void DropperServer::goalCallback(const auv_common::DropperGoalConstPtr &goal) {

    auv_common::DropperCmd dropperCmd;
    dropperCmd.request.velocity = -127;
    ros::service::call(dropperService, dropperCmd);
    ros::Duration(3).sleep();

    dropperCmd.request.velocity = 127;
    ros::service::call(dropperService, dropperCmd);
    ros::Duration(3).sleep();

    dropperCmd.request.velocity = 0;
    ros::service::call(dropperService, dropperCmd);

    actionServer.setSucceeded();
}