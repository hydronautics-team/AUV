#include <DiveActionServer.h>
#include <auv_common/DepthCmd.h>
#include <std_msgs/UInt32.h>


DiveActionServer::DiveActionServer(const std::string &actionName, const std::string &depthService,
        const std::string& depthTopic, unsigned int depthRange) :
        depthService(depthService),
        depthTopic(depthTopic),
        depthRange(depthRange),
        actionServer(nodeHandle, actionName, boost::bind(&DiveActionServer::goalCallback, this, _1), false) {
    actionServer.start();
}


void DiveActionServer::goalCallback(const auv_common::DiveGoalConstPtr &goal) {
    unsigned int desiredDepth = goal->depth;

    auv_common::DepthCmd cmd;
    cmd.request.depth = goal->depth;
    ros::service::call(depthService, cmd); // TODO: Check response

    bool depthInRange = false;
    while (!depthInRange) {
        std_msgs::UInt32 depthMessage = *ros::topic::waitForMessage<std_msgs::UInt32>(depthTopic, nodeHandle);
        unsigned int currentDepth = depthMessage.data;
        if (std::abs(desiredDepth - currentDepth) < depthRange)
            depthInRange = true;
    }

    actionServer.setSucceeded();
}