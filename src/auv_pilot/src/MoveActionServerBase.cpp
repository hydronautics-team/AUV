
#include <MoveActionServerBase.h>

#include "../include/MoveActionServerBase.h"

MoveActionServerBase::MoveActionServerBase(const std::string& actionName, const std::string& velocityTopic,
        const TwistFactory& twistFactory):
        actionServer(nodeHandle, actionName, boost::bind(&MoveActionServerBase::goalCallback, this, _1), false),
        velocityPublishTopic(velocityTopic) {
    this->twistFactory = const_cast<TwistFactory*>(&twistFactory);
    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(velocityTopic, VELOCITY_TOPIC_QUEUE_SIZE);
    actionServer.start();
}

geometry_msgs::Twist MoveActionServerBase::createTwistFromGoal(const auv_common::MoveGoal &goal) {
    Direction direction;
    switch (goal.direction) {
        case auv_common::MoveGoal::DIRECTION_FORWARD:
            direction = Direction::FORWARD;
            break;

        case auv_common::MoveGoal::DIRECTION_BACKWARDS:
            direction = Direction::BACKWARDS;
            break;

        case auv_common::MoveGoal::DIRECTION_RIGHT:
            direction = Direction::RIGHT;
            break;

        case auv_common::MoveGoal::DIRECTION_LEFT:
            direction = Direction::LEFT;
            break;

        case auv_common::MoveGoal::DIRECTION_UP:
            direction = Direction::UP;
            break;

        case auv_common::MoveGoal::DIRECTION_DOWN:
            direction = Direction::DOWN;
            break;

        case auv_common::MoveGoal::ROTATE_ROLL_CW:
            direction = Direction::ROLL_CW;
            break;

        case auv_common::MoveGoal::ROTATE_ROLL_CCW:
            direction = Direction::ROLL_CCW;
            break;

        case auv_common::MoveGoal::ROTATE_PITCH_CW:
            direction = Direction::PITCH_CW;
            break;

        case auv_common::MoveGoal::ROTATE_PITCH_CCW:
            direction = Direction::PITCH_CCW;
            break;

        case auv_common::MoveGoal::ROTATE_YAW_CW:
            direction = Direction::YAW_CW;
            break;

        case auv_common::MoveGoal::ROTATE_YAW_CCW:
            direction = Direction::YAW_CCW;
            break;

        default:
            direction = Direction::STOP;
            break;
    }

    if (direction == Direction::STOP)
        return twistFactory->createStopTwist();

    if (goal.velocityLevel == auv_common::MoveGoal::VELOCITY_CUSTOM)
        return twistFactory->createDirectionTwist(direction, goal.velocityValue);

    switch (goal.velocityLevel) {
        case auv_common::MoveGoal::VELOCITY_LEVEL_1:
            return twistFactory->createDirectionTwist(direction, VelocityLevel::LEVEL_1);
        case auv_common::MoveGoal::VELOCITY_LEVEL_2:
            return twistFactory->createDirectionTwist(direction, VelocityLevel::LEVEL_2);
        case auv_common::MoveGoal::VELOCITY_LEVEL_3:
            return twistFactory->createDirectionTwist(direction, VelocityLevel::LEVEL_3);
        case auv_common::MoveGoal::VELOCITY_LEVEL_4:
            return twistFactory->createDirectionTwist(direction, VelocityLevel::LEVEL_4);
        default: /* Must not appear */
            return twistFactory->createStopTwist();
    }
}

void MoveActionServerBase::safePublish(const geometry_msgs::Twist& twist) {
    ros::Rate pollRate(VELOCITY_TOPIC_POLL_RATE);
    while (velocityPublisher.getNumSubscribers() == 0)
        pollRate.sleep();
    velocityPublisher.publish(twist);
}