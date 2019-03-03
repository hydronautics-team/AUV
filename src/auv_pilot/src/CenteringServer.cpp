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
    auv_common::VelocityCmd velocityCmd;
    velocityCmd.request.twist = twist;
    ros::service::call(velocityService, velocityCmd);
}

void CenteringServer::goalCallback(const auv_common::CenteringGoalConstPtr &goal) {

    std::string topic = goal->targetSource;

    bool inRange = false;
    boost::function<void (const auv_common::Gate::ConstPtr&)> callback =
            [&inRange, this] (const auv_common::Gate::ConstPtr& gateMsg) {

                if (inRange || !(gateMsg->isPresent))
                    return;

                double limit = (gateMsg->xTR - gateMsg->xTL) / 4.0;
                ROS_INFO("Center: %f, Limit: %f", gateMsg->xCenter, limit);
                if (std::abs(gateMsg->xCenter) < limit) {
                    this->move(Direction::STOP);
                    inRange = true;
                } else {
                    Direction direction = gateMsg->xCenter > 0 ? Direction::RIGHT : Direction::LEFT;
                    this->move(direction);
                }
            };

    ros::Subscriber sub = nodeHandle.subscribe("/gate", 1, callback);
    while (!inRange) { }
    sub.shutdown();

    actionServer.setSucceeded();
}