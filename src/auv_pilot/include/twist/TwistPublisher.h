#ifndef AUV_PILOT_TWISTPUBLISHER_H
#define AUV_PILOT_TWISTPUBLISHER_H

#include <geometry_msgs/Twist.h>

class TwistPublisher {

public:

    virtual bool publishTwist(const geometry_msgs::Twist& twist) = 0;

};


#endif //AUV_PILOT_TWISTPUBLISHER_H
