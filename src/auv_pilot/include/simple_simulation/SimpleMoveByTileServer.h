#ifndef AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTILESERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>


/**
 * Action server for apparatus movement
 * using navigation by tile on the bottom of the pool
 * in simple Gazebo simulation.
 */
class SimpleMoveByTileServer : public SimpleMoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    SimpleMoveByTileServer(const std::string& actionName);
    ~SimpleMoveByTileServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
