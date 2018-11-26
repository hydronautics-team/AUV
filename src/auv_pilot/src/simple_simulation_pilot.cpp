#include <ros/ros.h>
#include <string>
#include <simple_simulation/SimpleMoveByTimeServer.h>
#include <simple_simulation/SimpleMoveByTileServer.h>
#include <simple_simulation/SimpleMoveCenteringServer.h>

static const std::string SIMPLE_SIMULATION_PILOT_NODE_NAME = "simple_simulation_pilot";

static const std::string MOVE_BY_TIME_ACTION = "move_by_time";

static const std::string MOVE_BY_TILE_ACTION = "move_by_tile";

static const std::string MOVE_CENTERING = "move_centering";

int main(int argc, char **argv)
{
    ros::init(argc, argv, SIMPLE_SIMULATION_PILOT_NODE_NAME);
    SimpleMoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION);
    SimpleMoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION);
    SimpleMoveCenteringServer moveCenteringServer(MOVE_CENTERING);
    ros::spin();
}