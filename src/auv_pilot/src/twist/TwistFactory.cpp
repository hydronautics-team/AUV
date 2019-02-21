#include "../../include/twist/TwistFactory.h"

TwistFactory::TwistFactory(float velocityLevel1Value, float velocityLevel2Value, float velocityLevel3Value,
        float velocityLevel4Value) : velocityLevel1Value (velocityLevel1Value),
                                     velocityLevel2Value (velocityLevel2Value),
                                     velocityLevel3Value (velocityLevel3Value),
                                     velocityLevel4Value (velocityLevel4Value) {}

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

geometry_msgs::Twist TwistFactory::createDirectionTwist(Direction direction, VelocityLevel velocityLevel) {
    switch (velocityLevel) {
        case VelocityLevel::LEVEL_1:
            return createDirectionTwist(direction, velocityLevel1Value);
        case VelocityLevel::LEVEL_2:
            return createDirectionTwist(direction, velocityLevel2Value);
        case VelocityLevel::LEVEL_3:
            return createDirectionTwist(direction, velocityLevel3Value);
        case VelocityLevel::LEVEL_4:
            return createDirectionTwist(direction, velocityLevel4Value);
    }
}

geometry_msgs::Twist TwistFactory::createStopTwist() {
    return createLinearTwist(0.0f, 0.0f, 0.0f);
}