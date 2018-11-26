#include <ros/ros.h>
#include <string>
#include <simple_simulation/SimpleMoveByTimeServer.h>
#include <simple_simulation/SimpleMoveByTileServer.h>

static const std::string SIMPLE_SIMULATION_PILOT_NODE_NAME = "simple_simulation_pilot";

static const std::string MOVE_BY_TIME_ACTION = "simple_move_by_time";

static const std::string MOVE_BY_TILE_ACTION = "simple_move_by_tile";

int main(int argc, char **argv)
{
    ros::init(argc, argv, SIMPLE_SIMULATION_PILOT_NODE_NAME);
    SimpleMoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION);
    SimpleMoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION);
    ros::spin();
}