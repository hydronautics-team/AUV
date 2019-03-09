#include "CenteringServer.h"
#include <auv_common/Gate.h>

CenteringServer::CenteringServer(const std::string& actionName, const std::string& velocityService,
        const TwistFactory& twistFactory) :
        velocityService(velocityService),
        actionServer(nodeHandle, actionName, boost::bind(&CenteringServer::goalCallback, this, _1), false) {
    this->twistFactory = const_cast<TwistFactory*>(&twistFactory);
    actionServer.start();
}

void CenteringServer::move(const Direction &direction) {
    geometry_msgs::Twist twist = twistFactory->createDirectionTwist(direction, VelocityLevel::LEVEL_1);
    twist.linear.x = std::abs(twist.linear.z);
    auv_common::VelocityCmd velocityCmd;
    velocityCmd.request.twist = twist;
    ros::service::call(velocityService, velocityCmd);
}

void CenteringServer::goalCallback(const auv_common::CenteringGoalConstPtr &goal) {

    std::string topic = goal->targetSource;

    bool inRange = false;
    int negatives = 0;
    int lagChanges = 0;
    
    boost::function<void (const auv_common::Gate::ConstPtr&)> callback =
            [&inRange, &negatives, &lagChanges, this] (const auv_common::Gate::ConstPtr& gateMsg) {

                if (inRange || !(gateMsg->isPresent)) {
                    negatives++;
                    if (negatives > 15) {
                        ROS_INFO("Too many gate missings, asumming we have entered the gates");
                        inRange = true;
                    }
                    return;
                }

                //double limit = (gateMsg->xTR - gateMsg->xTL) / 4.0;
                //ROS_INFO("Center: %f, Limit: %f", gateMsg->xCenter, limit);

                if (std::abs(gateMsg->xCenter) < 10.0) {
                    this->move(Direction::STOP);
                    inRange = true;
                } else {
                    if (lagChanges > 15) {
                        this->move(Direction::STOP);
                        inRange = true;
                    }
                    Direction direction = gateMsg->xCenter > 0 ? Direction::RIGHT : Direction::LEFT;
                    lagChanges++;
                    this->move(direction);
                }
            };

    ros::Subscriber sub = nodeHandle.subscribe("/gate", 1, callback);
    while (!inRange) { }
    sub.shutdown();

    actionServer.setSucceeded();
}