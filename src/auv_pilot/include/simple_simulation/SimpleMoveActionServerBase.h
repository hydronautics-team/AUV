#ifndef AUV_PILOT_SIMPLEMOVEACTIONSERVERBASE_H
#define AUV_PILOT_SIMPLEMOVEACTIONSERVERBASE_H

#include <common/MoveActionServerBase.h>


/**
 * Base abstract class for implementing
 * action servers for simple simulation in Gazebo.
 */
class SimpleMoveActionServerBase : public MoveActionServerBase {

protected:

    static constexpr const char* GAZEBO_VELOCITY_TOPIC = "/cmd_vel";

    static const int GAZEBO_VELOCITY_TOPIC_QUEUE_SIZE = 1;

    static const int GAZEBO_VELOCITY_TOPIC_POLL_RATE = 100;

    ros::Publisher velocityPublisher;

    /** Publishes message to Gazebo velocity topic.<br>
     *  Gazebo simulation requires a bit special publishing mechanism
     *  (see implementation).
     */
    void publishToGazebo(const geometry_msgs::Twist& twist);

    virtual void goalCallback(const auv_common::MoveGoalConstPtr &goal) = 0;

public:

    SimpleMoveActionServerBase(const std::string& actionName);
    ~SimpleMoveActionServerBase() = default;

};

#endif //AUV_PILOT_SIMPLEMOVEACTIONSERVERBASE_H
