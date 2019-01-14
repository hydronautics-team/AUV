#ifndef AUV_PILOT_TWISTFACTORY_H
#define AUV_PILOT_TWISTFACTORY_H

#include <geometry_msgs/Twist.h>


/**
 * Base class for Twist instances
 * creation, based on different logic.
 *
 * This class is designed to implement
 * twist creation logic in derived classes
 * via implementing virtual methods.
 *
 * This class should not be instantiated!
 */
class TwistFactory {

public:

    TwistFactory() = default;
    TwistFactory(TwistFactory& other) = default;
    ~TwistFactory() = default;

    TwistFactory& operator=(const TwistFactory& other) = default;

    /** Initializes twist message */
    geometry_msgs::Twist createTwist(float x, float y, float z, float roll, float pitch, float yaw);

    /** Initializes twist message with zero angular velocities */
    geometry_msgs::Twist createLinearTwist(float x, float y, float z);

    /** Initializes twist message with zero linear velocities */
    geometry_msgs::Twist createAngularTwist(float roll, float pitch, float yaw);

    /** Initializes twist message for specified direction */
    virtual geometry_msgs::Twist createDirectionTwist(int direction, float velocity);
    virtual geometry_msgs::Twist createDirectionTwist(int direction);

};


#endif //AUV_PILOT_ITWISTFACTORY_H
