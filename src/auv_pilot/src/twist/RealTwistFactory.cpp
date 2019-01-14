#include <auv_common/MoveGoal.h>
#include "../../include/twist/RealTwistFactory.h"


RealTwistFactory::RealTwistFactory() : TwistFactory() {}


RealTwistFactory::RealTwistFactory(TwistFactory &other) : TwistFactory(other) {}


/* TODO: Fix all direction */
geometry_msgs::Twist RealTwistFactory::createDirectionTwist(int direction, float velocity) {
    switch (direction) {
        case auv_common::MoveGoal::DIRECTION_FORWARD:
            return createLinearTwist(velocity, 0.0f, 0.0f);

        case auv_common::MoveGoal::DIRECTION_BACKWARDS:
            return createLinearTwist(-velocity, 0.0f, 0.0f);

        case auv_common::MoveGoal::DIRECTION_RIGHT:
            return createLinearTwist(0.0f, velocity, 0.0f);

        case auv_common::MoveGoal::DIRECTION_LEFT:
            return createLinearTwist(0.0f, -velocity, 0.0f);

        case auv_common::MoveGoal::ROTATE_ROLL_CW:
            return createAngularTwist(velocity, 0.0f, 0.0f);

        case auv_common::MoveGoal::ROTATE_ROLL_CCW:
            return createAngularTwist(-velocity, 0.0f, 0.0f);

        case auv_common::MoveGoal::ROTATE_PITCH_CW:
            return createAngularTwist(0.0f, velocity, 0.0f);

        case auv_common::MoveGoal::ROTATE_PITCH_CCW:
            return createAngularTwist(0.0f, -velocity, 0.0f);

        case auv_common::MoveGoal::ROTATE_YAW_CW:
            return createAngularTwist(0.0f, velocity, 0.0f); // This one is only correct

        case auv_common::MoveGoal::ROTATE_YAW_CCW:
            return createAngularTwist(0.0f, -velocity, 0.0f);

        default:
            return createLinearTwist(0.0f, 0.0f, 0.0f);
    }
}


geometry_msgs::Twist RealTwistFactory::createDirectionTwist(int direction) {
    return createDirectionTwist(direction, DEFAULT_VELOCITY);
}