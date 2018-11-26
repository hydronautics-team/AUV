#ifndef AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
#define AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H

#include <common/MoveActionServerBase.h>

class SimpleMoveCenteringServer : MoveActionServerBase {

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveCenteringServer(const std::string& actionName);
    ~SimpleMoveCenteringServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVECENTERINGSERVER_H
