#include <auv_common/MoveGoal.h>
#include "../../include/twist/RealTwistFactory.h"


RealTwistFactory::RealTwistFactory(float velocityLevel1Value, float velocityLevel2Value, float velocityLevel3Value,
        float velocityLevel4Value) : TwistFactory(velocityLevel1Value, velocityLevel2Value, velocityLevel3Value,
                velocityLevel4Value) {}


RealTwistFactory::RealTwistFactory(TwistFactory &other) : TwistFactory(other) {}


geometry_msgs::Twist RealTwistFactory::createDirectionTwist(Direction direction, float velocity) {
    switch (direction) {
        case Direction::FORWARD:
            return createLinearTwist(velocity, 0.0f, 0.0f);

        case Direction::BACKWARDS:
            return createLinearTwist(-velocity, 0.0f, 0.0f);

        case Direction::RIGHT:
            return createLinearTwist(0.0f, 0.0f, velocity);

        case Direction::LEFT:
            return createLinearTwist(0.0f, 0.0f, -velocity);

        case Direction::UP:
            return createLinearTwist(0.0f, velocity, 0.0f);

        case Direction::DOWN:
            return createLinearTwist(0.0f, -velocity, 0.0f);

        case Direction::ROLL_CW:
            return createAngularTwist(velocity, 0.0f, 0.0f);

        case Direction::ROLL_CCW:
            return createAngularTwist(-velocity, 0.0f, 0.0f);

        case Direction::PITCH_CW:
            return createAngularTwist(0.0f, 0.0f, velocity);

        case Direction::PITCH_CCW:
            return createAngularTwist(0.0f, 0.0f, -velocity);

        case Direction::YAW_CW:
            return createAngularTwist(0.0f, velocity, 0.0f);

        case Direction::YAW_CCW:
            return createAngularTwist(0.0f, -velocity, 0.0f);

        case Direction::STOP:
            return createLinearTwist(0.0f, 0.0f, 0.0f);
    }
}
