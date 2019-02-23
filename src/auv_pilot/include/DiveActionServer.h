#ifndef AUV_PILOT_DIVEACTIONSERVER_H
#define AUV_PILOT_DIVEACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/DiveAction.h>

class DiveActionServer {

private:

    std::string depthService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::DiveAction> actionServer;

    void goalCallback(const auv_common::DiveGoalConstPtr &goal);

public:

    DiveActionServer(const std::string& actionName, const std::string& depthService);
    ~DiveActionServer() = default;
};


#endif //AUV_PILOT_DIVEACTIONSERVER_H
