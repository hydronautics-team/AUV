#ifndef AUV_PILOT_MOVEACTIONSERVERBASE_H
#define AUV_PILOT_MOVEACTIONSERVERBASE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/MoveAction.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>


/**
 * Base abstract class for implementing
 * action servers for apparatus movement.
 */
class MoveActionServerBase {

protected:

    static constexpr const float DEFAULT_VELOCITY = 0.5;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    /** Initializes twist message */
    geometry_msgs::Twist createTwist(float x, float y, float z, float roll, float pitch, float yaw);

    /** Initializes twist message with zero angular velocities */
    geometry_msgs::Twist createLinearTwist(float x, float y, float z);

    /** Initializes twist message for specified direction */
    geometry_msgs::Twist createDirectionTwist(int direction, float velocity = DEFAULT_VELOCITY);

    /** Implement your goal processing logic */
    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
