#ifndef AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>

class SimpleMoveByTimeServer : public SimpleMoveActionServerBase {

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveByTimeServer(const std::string& actionName);
    ~SimpleMoveByTimeServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
