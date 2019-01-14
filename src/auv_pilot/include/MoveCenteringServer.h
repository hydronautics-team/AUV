#ifndef AUV_PILOT_MOVECENTERING_H
#define AUV_PILOT_MOVECENTERING_H

#include <MoveActionServerBase.h>


/**
 * Action server for apparatus stabilization
 * using center of recognized object.
 */
class MoveCenteringServer : MoveActionServerBase {

protected:

    void goalCallback(const auv_common::MoveGoalConstPtr &goal);

public:

    MoveCenteringServer(const std::string& actionName, const std::string& velocityTopic, const TwistFactory& twistFactory);
    ~MoveCenteringServer() = default;

};

#endif //AUV_PILOT_MOVECENTERING_H
