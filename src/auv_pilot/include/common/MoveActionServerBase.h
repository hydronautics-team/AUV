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

    static constexpr const float DEFAULT_VELOCITY = 0.5;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    geometry_msgs::Twist createTwist(float x, float y, float z, float roll, float pitch, float yaw);
    geometry_msgs::Twist createLinearTwist(float x, float y, float z);
    geometry_msgs::Twist createDirectionTwist(int direction, float velocity = DEFAULT_VELOCITY);

    virtual void executeCallback(const auv_common::MoveGoalConstPtr& goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
