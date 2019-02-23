#include <DiveActionServer.h>
#include <auv_common/DepthCmd.h>


DiveActionServer::DiveActionServer(const std::string &actionName, const std::string &depthService) :
        depthService(depthService),
        actionServer(nodeHandle, actionName, boost::bind(&DiveActionServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void DiveActionServer::goalCallback(const auv_common::DiveGoalConstPtr &goal) {
    auv_common::DepthCmd cmd;
    cmd.request.depth = goal->depth;
    ros::service::call(depthService, cmd); // TODO: Check response

    ros::Duration(5.0).sleep();

    actionServer.setSucceeded();
}