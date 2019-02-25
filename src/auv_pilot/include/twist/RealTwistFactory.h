#ifndef AUV_PILOT_REALTWISTFACTORY_H
#define AUV_PILOT_REALTWISTFACTORY_H

#include "TwistFactory.h"


/**
 * Twist factory for real apparatus
 */
class RealTwistFactory : public TwistFactory {

public:

    RealTwistFactory(float velocityLevel1Value, float velocityLevel2Value, float velocityLevel3Value, float velocityLevel4Value);
    explicit RealTwistFactory(TwistFactory& other);
    ~RealTwistFactory() = default;

    geometry_msgs::Twist createDirectionTwist(Direction direction, float velocity) override;

};


#endif //AUV_PILOT_REALTWISTFACTORY_H
