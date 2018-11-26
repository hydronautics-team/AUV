#ifndef AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
#define AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H

#include <common/MoveActionServerBase.h>

class SimpleMoveByTimeServer : MoveActionServerBase {

private:

    ros::Publisher velocityPublisher;

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    SimpleMoveByTimeServer(const std::string& actionName);
    ~SimpleMoveByTimeServer() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEBYTIMESERVER_H
