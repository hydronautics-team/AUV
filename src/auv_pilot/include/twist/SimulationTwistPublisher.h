#ifndef AUV_PILOT_SIMULATIONTWISTPUBLISHER_H
#define AUV_PILOT_SIMULATIONTWISTPUBLISHER_H

#include <ros/ros.h>
#include "TwistPublisher.h"

class SimulationTwistPublisher : public TwistPublisher {

private:

    static const int VELOCITY_TOPIC_QUEUE_SIZE = 1;

    static const int VELOCITY_TOPIC_POLL_RATE = 100;

    ros::Publisher velocityPublisher;

public:

    SimulationTwistPublisher(ros::NodeHandle& nodeHandle, const std::string& velocityTopic);
    SimulationTwistPublisher(SimulationTwistPublisher& other) = default;
    ~SimulationTwistPublisher() = default;

    bool publishTwist(const geometry_msgs::Twist& twist) override;

};

#endif //AUV_PILOT_SIMULATIONTWISTPUBLISHER_H
