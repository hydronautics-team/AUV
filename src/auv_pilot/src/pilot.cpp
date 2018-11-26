#include <ros/ros.h>
#include <string>
#include <real/MoveByTimeServer.h>
#include <real/MoveByTileServer.h>

static const std::string PILOT_NODE_NAME = "pilot";

static const std::string MOVE_BY_TIME_ACTION = "move_by_time";

static const std::string MOVE_BY_TILE_ACTION = "move_by_tile";

int main(int argc, char **argv)
{
    ros::init(argc, argv, PILOT_NODE_NAME);
    MoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION);
    MoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION);
    ros::spin();
}