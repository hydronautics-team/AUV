#ifndef AUV_PILOT_MOVECENTERING_H
#define AUV_PILOT_MOVECENTERING_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <auv_common/CenteringAction.h>
#include <auv_common/VelocityCmd.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>
#include <twist/TwistFactory.h>


/**
 * Action server for apparatus stabilization
 * using center of recognized object.
 */
class CenteringServer {

private:

    /* TODO: Try to use persistent connection */
    std::string velocityService;

    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<auv_common::MoveAction> actionServer;

    TwistFactory* twistFactory;

    void goalCallback(const auv_common::CenteringGoalConstPtr &goal);

public:

    CenteringServer(const std::string& actionName, const std::string& velocityService, const TwistFactory& twistFactory);
    ~CenteringServer() = default;

};


#endif //AUV_PILOT_MOVECENTERING_H
