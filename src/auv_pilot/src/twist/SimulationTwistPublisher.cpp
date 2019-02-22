#include <twist/SimulationTwistPublisher.h>

SimulationTwistPublisher::SimulationTwistPublisher(ros::NodeHandle &nodeHandle,
                                                   const std::string &velocityTopic) {
    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(velocityTopic, VELOCITY_TOPIC_QUEUE_SIZE);
}

bool SimulationTwistPublisher::publishTwist(const geometry_msgs::Twist &twist) {
    ros::Rate pollRate(VELOCITY_TOPIC_POLL_RATE);
    while (velocityPublisher.getNumSubscribers() == 0)
        pollRate.sleep();
    velocityPublisher.publish(twist);
    return true;
}

