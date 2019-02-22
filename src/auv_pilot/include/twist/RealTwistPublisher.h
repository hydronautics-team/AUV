#ifndef AUV_PILOT_REALTWISTPUBLISHER_H
#define AUV_PILOT_REALTWISTPUBLISHER_H

#include <ros/ros.h>
#include "TwistPublisher.h"

class RealTwistPublisher : public TwistPublisher {

private:

    ros::ServiceClient velocityClient;

public:

    RealTwistPublisher(ros::NodeHandle& nodeHandle, const std::string& velocityService);
    RealTwistPublisher(RealTwistPublisher& other) = default;
    ~RealTwistPublisher() = default;

    bool publishTwist(const geometry_msgs::Twist& twist);

};

#endif //AUV_PILOT_REALTWISTPUBLISHER_H
