#ifndef AUV_PILOT_MOVEACTIONSERVERBASE_H
#define AUV_PILOT_MOVEACTIONSERVERBASE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/MoveAction.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>
#include <twist/TwistFactory.h>
#include <twist/TwistPublisher.h>


/**
 * Base abstract class for implementing
 * action servers for apparatus movement.
 */
class MoveActionServerBase {

protected:

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    TwistFactory* twistFactory;
    TwistPublisher* twistPublisher;

    geometry_msgs::Twist createTwistFromGoal(const auv_common::MoveGoal &goal);

    /** Implement your goal processing logic */
    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    MoveActionServerBase(const std::string& actionName, bool isSimulation, const std::string& velocityTopicOrService,
            const TwistFactory& twistFactory);
    ~MoveActionServerBase();

};


#endif //AUV_PILOT_MOVEACTIONSERVERBASE_H
