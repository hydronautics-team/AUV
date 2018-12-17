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

    static constexpr const float DEFAULT_VELOCITY = 0.5f;

    static const int VELOCITY_TOPIC_QUEUE_SIZE = 1;

    static const int VELOCITY_TOPIC_POLL_RATE = 100;

    /** ROS topic to where velocity values will be published */
    std::string velocityPublishTopic;

    ros::Publisher velocityPublisher;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    /** Initializes twist message */
    geometry_msgs::Twist createTwist(float x, float y, float z, float roll, float pitch, float yaw);

    /** Initializes twist message with zero angular velocities */
    geometry_msgs::Twist createLinearTwist(float x, float y, float z);

    /** Initializes twist message with zero linear velocities */
    geometry_msgs::Twist createAngularTwist(float roll, float pitch, float yaw);

    /** Initializes twist message for specified direction */
    geometry_msgs::Twist createDirectionTwist(int direction, float velocity = DEFAULT_VELOCITY);

    void safePublish(const geometry_msgs::Twist &twist);

    /** Implement your goal processing logic */
    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName, const std::string velocityTopic);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
