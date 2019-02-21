#ifndef AUV_PILOT_MOVEACTIONSERVERBASE_H
#define AUV_PILOT_MOVEACTIONSERVERBASE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/MoveAction.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>
#include <twist/TwistFactory.h>


/**
 * Base abstract class for implementing
 * action servers for apparatus movement.
 */
class MoveActionServerBase {

protected:

    static const int VELOCITY_TOPIC_QUEUE_SIZE = 1;

    static const int VELOCITY_TOPIC_POLL_RATE = 100;

    /** ROS topic to where velocity values will be published */
    std::string velocityPublishTopic;

    ros::Publisher velocityPublisher;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    TwistFactory* twistFactory;

    void safePublish(const geometry_msgs::Twist &twist);

    geometry_msgs::Twist createTwistFromGoal(const auv_common::MoveGoal &goal);

    /** Implement your goal processing logic */
    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
