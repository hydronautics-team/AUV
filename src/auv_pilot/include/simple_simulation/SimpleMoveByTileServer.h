#ifndef AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTILESERVER_H

#include <common/MoveActionServerBase.h>

class SimpleMoveByTileServer : MoveActionServerBase {

private:

    ros::Publisher velocityPublisher;

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveByTileServer(const std::string& actionName);
    ~SimpleMoveByTileServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTILESERVER_H
