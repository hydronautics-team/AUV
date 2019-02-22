#include "MoveByTimeServer.h"
#include <iostream>

MoveByTimeServer::MoveByTimeServer(const std::string& actionName, bool isSimulation, const std::string& velocityTopicOrService,
                                   const TwistFactory& twistFactory):
    MoveActionServerBase(actionName, isSimulation, velocityTopicOrService, twistFactory) { };

void MoveByTimeServer::goalCallback(const auv_common::MoveGoalConstPtr &goal) {

    geometry_msgs::Twist startMsg = createTwistFromGoal(*goal);

    /* TODO: check if publish returned false */
    twistPublisher->publishTwist(startMsg);

    if (goal->value != auv_common::MoveGoal::VALUE_INFINITY) {
        int timeMs = goal->value;
        float time = timeMs / 1000.0f;
        if (time != 0) {
            ros::Duration(time).sleep();
            geometry_msgs::Twist endMsg = twistFactory->createStopTwist();
            twistPublisher->publishTwist(endMsg);
            ros::Duration(2.0).sleep();
        }
    } else if (goal->holdIfInfinityValue) {
        while (true)
        {
        }
    }

    actionServer.setSucceeded();
}