#ifndef PROJECT_LIFTERSERVICE_H
#define PROJECT_LIFTERSERVICE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/LifterAction.h>

class LifterServer {

private:

    std::string lifterService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::LifterAction> actionServer;

    void goalCallback(const auv_common::LifterGoalConstPtr &goal);

public:

    LifterServer(const std::string& actionName, const std::string& lifterService);
    ~LifterServer() = default;
};

#endif //PROJECT_LIFTERSERVICE_H
