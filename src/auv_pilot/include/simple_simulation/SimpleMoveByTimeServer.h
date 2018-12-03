#ifndef AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>


/**
 * Action server for apparatus movement
 * using time delays in simple Gazebo simulation.
 */
class SimpleMoveByTimeServer : public SimpleMoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    SimpleMoveByTimeServer(const std::string& actionName);
    ~SimpleMoveByTimeServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
