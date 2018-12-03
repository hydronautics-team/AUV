#include <common/MoveActionServerBase.h>

MoveActionServerBase::MoveActionServerBase(const std::string &actionName):
        actionServer(nodeHandle, actionName, boost::bind(&MoveActionServerBase::goalCallback, this, _1), false) {
    actionServer.start();
}

geometry_msgs::Twist MoveActionServerBase::createTwist(float x, float y, float z, float roll, float pitch, float yaw) {
    geometry_msgs::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.linear.z = z;
    twist.angular.x = roll;
    twist.angular.y = pitch;
    twist.angular.z = yaw;
    return twist;
}

geometry_msgs::Twist MoveActionServerBase::createLinearTwist(float x, float y, float z) {
    return createTwist(x, y, z, 0.0, 0.0, 0.0);
}

geometry_msgs::Twist MoveActionServerBase::createDirectionTwist(int direction, float velocity) {
    switch (direction) {
        case auv_common::MoveGoal::DIRECTION_FORWARD:
            return createLinearTwist(-velocity, 0, 0); // Workaraound for current model
        case auv_common::MoveGoal::DIRECTION_BACKWARDS:
            return createLinearTwist(velocity, 0, 0);
        case auv_common::MoveGoal::DIRECTION_RIGHT:
            return createLinearTwist(0, velocity, 0);
        case auv_common::MoveGoal::DIRECTION_LEFT:
            return createLinearTwist(0, -velocity, 0);
        default:
            return createLinearTwist(0, 0, 0);
    }
}