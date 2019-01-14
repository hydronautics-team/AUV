#ifndef AUV_PILOT_SIMULATIONTWISTFACTORY_H
#define AUV_PILOT_SIMULATIONTWISTFACTORY_H

#include "TwistFactory.h"


/**
 * Twist factory for the simple simulator.
 */
class SimulationTwistFactory : public TwistFactory {

private:

    static constexpr const float DEFAULT_VELOCITY = 10.0f;

public:

    SimulationTwistFactory();
    SimulationTwistFactory(TwistFactory& other);
    ~SimulationTwistFactory() = default;

    geometry_msgs::Twist createDirectionTwist(int direction, float velocity);
    geometry_msgs::Twist createDirectionTwist(int direction);

};


#endif //AUV_PILOT_SIMULATIONTWISTFACTORY_H
