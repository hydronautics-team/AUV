#ifndef AUV_PILOT_MOVEBYTILESERVER_H
#define AUV_PILOT_MOVEBYTILESERVER_H

#include <common/MoveActionServerBase.h>

class MoveByTileServer : MoveActionServerBase {

protected:

    void executeCallback(const auv_common::MoveGoalConstPtr& goal);

public:

    MoveByTileServer(const std::string& actionName);
    ~MoveByTileServer() = default;

};

#endif //AUV_PILOT_MOVEBYTILESERVER_H
