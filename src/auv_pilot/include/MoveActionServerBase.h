#ifndef AUV_PILOT_MOVEACTIONSERVERBASE_H
#define AUV_PILOT_MOVEACTIONSERVERBASE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/MoveAction.h>
#include <auv_common/VelocityCmd.h>
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

    /* TODO: Try to use persistent connection */
    std::string velocityService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    TwistFactory* twistFactory;

    geometry_msgs::Twist createTwistFromGoal(const auv_common::MoveGoal &goal);

    /** Implement your goal processing logic */
    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName, const std::string& velocityService, const TwistFactory& twistFactory);
    ~MoveActionServerBase() = default;

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
