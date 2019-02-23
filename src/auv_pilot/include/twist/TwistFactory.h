#ifndef AUV_PILOT_TWISTFACTORY_H
#define AUV_PILOT_TWISTFACTORY_H

#include <geometry_msgs/Twist.h>
#include <auv_common/MoveAction.h>

typedef enum {
    FORWARD,
    BACKWARDS,
    RIGHT,
    LEFT,
    ROLL_CW,
    ROLL_CCW,
    PITCH_CW,
    PITCH_CCW,
    YAW_CW,
    YAW_CCW,
    STOP
} Direction;

typedef enum {
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4
} VelocityLevel;

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

protected:

    float velocityLevel1Value;
    float velocityLevel2Value;
    float velocityLevel3Value;
    float velocityLevel4Value;

public:

    TwistFactory(float velocityLevel1Value, float velocityLevel2Value, float velocityLevel3Value, float velocityLevel4Value);
    TwistFactory(TwistFactory& other) = default;
    ~TwistFactory() = default;

    TwistFactory& operator=(const TwistFactory& other) = default;

    /** Initializes twist */
    geometry_msgs::Twist createTwist(float x, float y, float z, float roll, float pitch, float yaw);

    /** Initializes twist with zero angular velocities */
    geometry_msgs::Twist createLinearTwist(float x, float y, float z);

    /** Initializes twist with zero linear velocities */
    geometry_msgs::Twist createAngularTwist(float roll, float pitch, float yaw);

    /** Initializes twist for specified direction and velocity level */
    geometry_msgs::Twist createDirectionTwist(Direction direction, VelocityLevel velocityLevel);

    /** Creates twist to stop vehicle */
    geometry_msgs::Twist createStopTwist();

    /** Initializes twist for specified direction and velocity */
    virtual geometry_msgs::Twist createDirectionTwist(Direction direction, float velocity) = 0;

};


#endif //AUV_PILOT_ITWISTFACTORY_H
