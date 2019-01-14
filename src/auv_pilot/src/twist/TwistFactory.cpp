#include "../../include/twist/TwistFactory.h"


geometry_msgs::Twist TwistFactory::createTwist(float x, float y, float z, float roll, float pitch, float yaw) {
    geometry_msgs::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.linear.z = z;
    twist.angular.x = roll;
    twist.angular.y = pitch;
    twist.angular.z = yaw;
    return twist;
}

geometry_msgs::Twist TwistFactory::createLinearTwist(float x, float y, float z) {
    return createTwist(x, y, z, 0.0f, 0.0f, 0.0f);
}

geometry_msgs::Twist TwistFactory::createAngularTwist(float roll, float pitch, float yaw) {
    return createTwist(0.0f, 0.0f, 0.0f, roll, pitch, yaw);
}

geometry_msgs::Twist TwistFactory::createDirectionTwist(int direction, float velocity) {
    return createLinearTwist(0.0f, 0.0f, 0.0f);
}

geometry_msgs::Twist TwistFactory::createDirectionTwist(int direction) {
    return createLinearTwist(0.0f, 0.0f, 0.0f);
}