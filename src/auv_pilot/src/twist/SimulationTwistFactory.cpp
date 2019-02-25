#include <auv_common/MoveGoal.h>
#include "../../include/twist/SimulationTwistFactory.h"


SimulationTwistFactory::SimulationTwistFactory() : TwistFactory(0.3, 1.5, 2.3, 3.0) {}


SimulationTwistFactory::SimulationTwistFactory(TwistFactory &other) : TwistFactory(other) {}

geometry_msgs::Twist SimulationTwistFactory::createDirectionTwist(Direction direction, float velocity) {
    switch (direction) {
        case Direction::FORWARD:
            return createLinearTwist(velocity, 0.0f, 0.0f);

        case Direction::BACKWARDS:
            return createLinearTwist(-velocity, 0.0f, 0.0f);

        case Direction::RIGHT:
            return createLinearTwist(0.0f, -velocity, 0.0f);

        case Direction::LEFT:
            return createLinearTwist(0.0f, velocity, 0.0f);

        case Direction::ROLL_CW:
            return createAngularTwist(velocity, 0.0f, 0.0f);

        case Direction::ROLL_CCW:
            return createAngularTwist(-velocity, 0.0f, 0.0f);

        case Direction::PITCH_CW:
            return createAngularTwist(0.0f, velocity, 0.0f);

        case Direction::PITCH_CCW:
            return createAngularTwist(0.0f, -velocity, 0.0f);

        case Direction::YAW_CW:
            return createAngularTwist(0.0f, 0.0f, velocity);

        case Direction::YAW_CCW:
            return createAngularTwist(0.0f, 0.0f, -velocity);

        case Direction::STOP:
            return createLinearTwist(0.0f, 0.0f, 0.0f);
    }
}