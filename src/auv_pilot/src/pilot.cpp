#include <ros/ros.h>
#include <string>
#include <MoveByTimeServer.h>
#include <MoveByTileServer.h>
#include <MoveCenteringServer.h>
#include <twist/SimulationTwistFactory.h>
#include <twist/RealTwistFactory.h>

static const std::string PILOT_NODE_NAME = "pilot";

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
    nodeHandle.param("simulation", isSimulation, false);
    std::string topic = isSimulation ? GAZEBO_VELOCITY_TOPIC : REAL_VELOCITY_TOPIC;
    TwistFactory* twistFactory;
    if (isSimulation)
        twistFactory = new SimulationTwistFactory();
    else
        twistFactory = new RealTwistFactory();

    MoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION, topic, *twistFactory);
    MoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION, topic, *twistFactory);
    MoveCenteringServer moveCenteringServer(MOVE_CENTERING, topic, *twistFactory);
    ros::spin();

    delete twistFactory;
}