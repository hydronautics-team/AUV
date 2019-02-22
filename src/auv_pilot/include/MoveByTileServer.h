#ifndef AUV_PILOT_MOVEBYTILESERVER_H
#define AUV_PILOT_MOVEBYTILESERVER_H

#include <MoveActionServerBase.h>


/**
 * Action server for apparatus movement
 * using navigation by tile on the bottom of the pool.
 */
class MoveByTileServer : MoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal) override;

public:

    MoveByTileServer(const std::string& actionName, bool isSimulation, const std::string& velocityTopicOrService,
            const TwistFactory& twistFactory);
    ~MoveByTileServer() = default;

};

#endif //AUV_PILOT_MOVEBYTILESERVER_H
