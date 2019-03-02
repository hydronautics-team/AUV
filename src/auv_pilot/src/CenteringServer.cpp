#include "CenteringServer.h"
#include <auv_common/OptionalPoint2D.h>

CenteringServer::CenteringServer(const std::string& actionName, const std::string& velocityService,
        const TwistFactory& twistFactory) :
        velocityService(velocityService),
        actionServer(nodeHandle, actionName, boost::bind(&CenteringServer::goalCallback, this, _1), false) {
    this->twistFactory = const_cast<TwistFactory*>(&twistFactory);
    actionServer.start();
}

void CenteringServer::goalCallback(const auv_common::CenteringGoalConstPtr &goal) {

    std::string topic = goal->targetSource;

    Direction direction;
    auv_common::OptionalPoint2D firstPoint = *ros::topic::waitForMessage<auv_common::OptionalPoint2D>(topic, nodeHandle);
    if (!firstPoint.hasPoint) {
        switch (goal->initialDirection) {
            case Direction::LEFT:
                direction = Direction::LEFT;
                break;
            case Direction::RIGHT:
                direction = Direction::RIGHT;
                break;
            default:
                actionServer.setAborted();
                return;
        }
    } else {
        if (std::abs(firstPoint.x) < goal->limits) {
            actionServer.setSucceeded();
            return;
        }
        direction = firstPoint.x > 0 ? Direction::RIGHT : Direction::LEFT;
    }

    bool inRange = false;
    while (!inRange) {



    }

    /** Temporary stub */
    actionServer.setSucceeded();
}