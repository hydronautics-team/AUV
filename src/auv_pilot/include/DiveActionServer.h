#ifndef AUV_PILOT_DIVEACTIONSERVER_H
#define AUV_PILOT_DIVEACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/DiveAction.h>

class DiveActionServer {

private:

    std::string depthService;

    std::string depthTopic;

    unsigned int depthRange;

    float diveTime;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::DiveAction> actionServer;

    void goalCallback(const auv_common::DiveGoalConstPtr &goal);

public:

    DiveActionServer(const std::string& actionName, const std::string& depthService, const std::string& depthTopic,
            unsigned int depthRange, float diveTime);
    ~DiveActionServer() = default;
};


#endif //AUV_PILOT_DIVEACTIONSERVER_H
