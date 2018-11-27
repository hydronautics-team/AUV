#ifndef AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
#define AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>

class SimpleMoveCenteringServer : public SimpleMoveActionServerBase {

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveCenteringServer(const std::string& actionName);
    ~SimpleMoveCenteringServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
