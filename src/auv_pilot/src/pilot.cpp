#include <ros/ros.h>
#include <string>
#include <MoveByTimeServer.h>
#include <MoveByTileServer.h>
#include <MoveCenteringServer.h>
#include <twist/SimulationTwistFactory.h>
#include <twist/RealTwistFactory.h>

static const std::string PILOT_NODE_NAME = "pilot";

static const std::string PARAM_SIMULTATION = "simulation";

static const std::string PARAM_REAL_VELOCITY = "realVelocity";

static const std::string PARAM_SIMULATION_VELOCITY = "simulationVelocity";

static const std::string GAZEBO_VELOCITY_TOPIC = "/cmd_vel";

static const std::string REAL_VELOCITY_TOPIC = "/pilot/velocity";

static const std::string MOVE_BY_TIME_ACTION = "move_by_time";

static const std::string MOVE_BY_TILE_ACTION = "move_by_tile";

static const std::string MOVE_CENTERING = "move_centering";

int main(int argc, char **argv)
{
    ros::init(argc, argv, PILOT_NODE_NAME);
    ros::NodeHandle nodeHandle(PILOT_NODE_NAME);

    bool isSimulation;
    float defaultVelocity;

    nodeHandle.param(PARAM_SIMULTATION, isSimulation, false);
    std::string topic = isSimulation ? GAZEBO_VELOCITY_TOPIC : REAL_VELOCITY_TOPIC;
    if (isSimulation)
        nodeHandle.param(PARAM_SIMULATION_VELOCITY, defaultVelocity, 0.0f);
    else
        nodeHandle.param(PARAM_REAL_VELOCITY, defaultVelocity, 0.0f);

    TwistFactory* twistFactory;
    if (isSimulation)
        twistFactory = new SimulationTwistFactory(defaultVelocity);
    else
        twistFactory = new RealTwistFactory(defaultVelocity);

    MoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION, topic, *twistFactory);
    MoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION, topic, *twistFactory);
    MoveCenteringServer moveCenteringServer(MOVE_CENTERING, topic, *twistFactory);
    ros::spin();

    delete twistFactory;
}