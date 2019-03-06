#ifndef PROJECT_DROPPERSERVER_H
#define PROJECT_DROPPERSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/DropperAction.h>

class DropperServer {

private:

    std::string dropperService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::DropperAction> actionServer;

    void goalCallback(const auv_common::DropperGoalConstPtr &goal);

public:

    DropperServer(const std::string& actionName, const std::string& dropperService);
    ~DropperServer() = default;
};

#endif //PROJECT_DROPPERSERVER_H
