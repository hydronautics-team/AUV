#ifndef AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTILESERVER_H

#include <simple_simulation/SimpleMoveActionServerBase.h>

class SimpleMoveByTileServer : public SimpleMoveActionServerBase {

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveByTileServer(const std::string& actionName);
    ~SimpleMoveByTileServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
