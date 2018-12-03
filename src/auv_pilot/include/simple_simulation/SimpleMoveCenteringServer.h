#ifndef AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
#define AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>


/**
 * Action server for apparatus stabilization
 * using center of recognized object in
 * simple Gazebo simulation.
 */
class SimpleMoveCenteringServer : public SimpleMoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    SimpleMoveCenteringServer(const std::string& actionName);
    ~SimpleMoveCenteringServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
