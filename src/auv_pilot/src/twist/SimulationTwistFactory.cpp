#include <auv_common/MoveGoal.h>
#include "../../include/twist/SimulationTwistFactory.h"


SimulationTwistFactory::SimulationTwistFactory() : TwistFactory() {}


SimulationTwistFactory::SimulationTwistFactory(TwistFactory &other) : TwistFactory(other) {}


geometry_msgs::Twist SimulationTwistFactory::createDirectionTwist(int direction, float velocity) {
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
            return createAngularTwist(0.0f, 0.0f, velocity);

        case auv_common::MoveGoal::ROTATE_YAW_CCW:
            return createAngularTwist(0.0f, 0.0f, -velocity);

        default:
            return createLinearTwist(0.0f, 0.0f, 0.0f);
    }
}


geometry_msgs::Twist SimulationTwistFactory::createDirectionTwist(int direction) {
    return createDirectionTwist(direction, DEFAULT_VELOCITY);
}