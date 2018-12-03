#ifndef AUV_PILOT_MOVECENTERING_H
#define AUV_PILOT_MOVECENTERING_H

#include <common/MoveActionServerBase.h>


/**
 * Action server for apparatus stabilization
 * using center of recognized object.
 */
class MoveCenteringServer : MoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    MoveCenteringServer(const std::string& actionName);
    ~MoveCenteringServer() = default;

};

#endif //AUV_PILOT_MOVECENTERING_H
