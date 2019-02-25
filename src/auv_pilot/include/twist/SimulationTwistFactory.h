#ifndef AUV_PILOT_SIMULATIONTWISTFACTORY_H
#define AUV_PILOT_SIMULATIONTWISTFACTORY_H

#include "TwistFactory.h"


/**
 * Twist factory for the simple simulator.
 */
class SimulationTwistFactory : public TwistFactory {

public:

    SimulationTwistFactory();
    explicit SimulationTwistFactory(TwistFactory& other);
    ~SimulationTwistFactory() = default;

    geometry_msgs::Twist createDirectionTwist(Direction direction, float velocity) override;

};


#endif //AUV_PILOT_SIMULATIONTWISTFACTORY_H
