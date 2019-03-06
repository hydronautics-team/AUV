#include <DiveActionServer.h>
#include <auv_common/DepthCmd.h>
#include <std_msgs/UInt32.h>


DiveActionServer::DiveActionServer(const std::string &actionName, const std::string &depthService,
        const std::string& depthTopic, unsigned int depthRange, float diveTime) :
        depthService(depthService),
        depthTopic(depthTopic),
        depthRange(depthRange),
        diveTime(diveTime),
        actionServer(nodeHandle, actionName, boost::bind(&DiveActionServer::goalCallback, this, _1), false) {
        actionServer.start();
}


void DiveActionServer::goalCallback(const auv_common::DiveGoalConstPtr &goal) {
    unsigned int desiredDepth = goal->depth;

    auv_common::DepthCmd cmd;
    cmd.request.depth = goal->depth;
    ros::service::call(depthService, cmd); // TODO: Check response

/*    ros::Rate sleepRate(1.0f / diveTime); // Sleep for 5 sec
    sleepRate.sleep();*/

    bool depthInRange = false;
    while (!depthInRange) {
        std_msgs::UInt32 depthMessage = *ros::topic::waitForMessage<std_msgs::UInt32>(depthTopic, nodeHandle);
        unsigned int currentDepth = depthMessage.data;
        ROS_INFO("Current depth: %d, desired depth: %d, depth error range: %d", currentDepth, desiredDepth, depthRange);
        if (std::abs(desiredDepth - currentDepth) < depthRange)
            depthInRange = true;
    }

    ROS_INFO("Dive succeeded");
    actionServer.setSucceeded();
}