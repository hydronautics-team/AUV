#include <ros/ros.h>
#include <string>
#include <MoveByTimeServer.h>
#include <MoveByTileServer.h>
#include <MoveCenteringServer.h>
#include <twist/SimulationTwistFactory.h>
#include <twist/RealTwistFactory.h>

static const std::string PILOT_NODE_NAME = "pilot";

static const std::string PARAM_SIMULTATION = "simulation";

static const std::string PARAM_VELOCITY_LEVEL_1 = "velocityLevel1";

static const std::string PARAM_VELOCITY_LEVEL_2 = "velocityLevel2";

static const std::string PARAM_VELOCITY_LEVEL_3 = "velocityLevel3";

static const std::string PARAM_VELOCITY_LEVEL_4 = "velocityLevel4";

static const std::string VELOCITY_SERVICE = "velocity_service";

static const std::string MOVE_BY_TIME_ACTION = "move_by_time";

static const std::string MOVE_BY_TILE_ACTION = "move_by_tile";

static const std::string MOVE_CENTERING = "move_centering";

int main(int argc, char **argv)
{
    ros::init(argc, argv, PILOT_NODE_NAME);
    ros::NodeHandle nodeHandle(PILOT_NODE_NAME);

    bool isSimulation;
    float defaultVelocity;

    TwistFactory* twistFactory;

    nodeHandle.param(PARAM_SIMULTATION, isSimulation, false);

    if (!isSimulation) {
        float velocity1, velocity2, velocity3, velocity4;
        /* TODO: Fix default values */
        nodeHandle.param(PARAM_VELOCITY_LEVEL_1, velocity1, 0.0f);
        nodeHandle.param(PARAM_VELOCITY_LEVEL_2, velocity2, 0.0f);
        nodeHandle.param(PARAM_VELOCITY_LEVEL_3, velocity3, 0.0f);
        nodeHandle.param(PARAM_VELOCITY_LEVEL_4, velocity4, 0.0f);
        twistFactory = new RealTwistFactory(velocity1, velocity2, velocity3, velocity4);
    } else {
        twistFactory = new SimulationTwistFactory();
    }

    MoveByTimeServer moveByTimeServer(MOVE_BY_TIME_ACTION, VELOCITY_SERVICE, *twistFactory);
    MoveByTileServer moveByTileServer(MOVE_BY_TILE_ACTION, VELOCITY_SERVICE, *twistFactory);
    MoveCenteringServer moveCenteringServer(MOVE_CENTERING, VELOCITY_SERVICE, *twistFactory);
    ros::spin();

    delete twistFactory;
}