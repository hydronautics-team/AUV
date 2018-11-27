#include <simple_simulation/SimpleMoveActionServerBase.h>

SimpleMoveActionServerBase::SimpleMoveActionServerBase(const std::string &actionName):
        MoveActionServerBase(actionName),
        velocityPublisher(nodeHandle.advertise<geometry_msgs::Twist>(GAZEBO_VELOCITY_TOPIC,
                GAZEBO_VELOCITY_TOPIC_QUEUE_SIZE)) { };

void SimpleMoveActionServerBase::publishToGazebo(const geometry_msgs::Twist& twist) {
    ros::Rate pollRate(GAZEBO_VELOCITY_TOPIC_POLL_RATE);
    while (velocityPublisher.getNumSubscribers() == 0)
        pollRate.sleep();
    velocityPublisher.publish(twist);
}