#ifndef AUV_PILOT_MOVEBYTILESERVER_H
#define AUV_PILOT_MOVEBYTILESERVER_H

#include <MoveActionServerBase.h>


/**
 * Action server for apparatus movement
 * using navigation by tile on the bottom of the pool.
 */
class MoveByTileServer : MoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    MoveByTileServer(const std::string& actionName, const std::string& velocityTopic);
    ~MoveByTileServer() = default;

};

#endif //AUV_PILOT_MOVEBYTILESERVER_H
