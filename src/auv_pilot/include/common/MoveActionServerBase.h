#ifndef AUV_PILOT_MOVEACTIONSERVERBASE_H
#define AUV_PILOT_MOVEACTIONSERVERBASE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/MoveAction.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>

class MoveActionServerBase {

protected:

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    virtual void executeCallback(const auv_common::MoveGoalConstPtr& goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
