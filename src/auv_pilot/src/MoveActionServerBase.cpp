
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


void MoveActionServerBase::safePublish(const geometry_msgs::Twist& twist) {
    ros::Rate pollRate(VELOCITY_TOPIC_POLL_RATE);
    while (velocityPublisher.getNumSubscribers() == 0)
        pollRate.sleep();
    velocityPublisher.publish(twist);
}