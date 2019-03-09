#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include <auv_common/VelocityCmd.h>
#include <auv_common/DepthCmd.h>
#include <auv_common/DropperCmd.h>
#include <auv_common/LifterCmd.h>
#include <auv_common/EnablingCmd.h>
#include <std_msgs/UInt32.h>

static const std::string GAZEBO_VELOCITY_TOPIC = "/cmd_vel";

static const std::string VELOCITY_SERVICE = "velocity_service";

static const std::string DEPTH_TOPIC = "/perception/depth";

static const std::string DEPTH_SERVICE = "depth_service";

ros::Publisher velocityPublisher;

ros::Publisher depthPublisher;

unsigned int currentDepth = 0;

geometry_msgs::Twist currentTwist;

bool movementCallback(auv_common::VelocityCmd::Request& velocityRequest,
                       auv_common::VelocityCmd::Response& velocityResponse);

bool depthCallback(auv_common::DepthCmd::Request& depthRequest,
                      auv_common::DepthCmd::Response& depthResponse);

bool dropperCallback(auv_common::DropperCmd::Request& dropperRequest,
                      auv_common::DropperCmd::Response& dropperResponse);

bool lifterCallback(auv_common::LifterCmd::Request& lifterRequest,
                    auv_common::LifterCmd::Response& lifterResponse);

bool imuInitCallback(auv_common::EnablingCmd::Request& enablingRequest,
                     auv_common::EnablingCmd::Response& enablingResponse);

bool stabilizationCallback(auv_common::EnablingCmd::Request& enablingRequest,
                           auv_common::EnablingCmd::Response& enablingResponse);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_bridge");
    ros::NodeHandle nodeHandle;

    velocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>(GAZEBO_VELOCITY_TOPIC, 100);
    depthPublisher = nodeHandle.advertise<std_msgs::UInt32>(DEPTH_TOPIC, 10);

    currentTwist.linear.x = currentTwist.linear.y = currentTwist.linear.z =
            currentTwist.angular.x = currentTwist.angular.y = currentTwist.angular.z = 0;

    ros::ServiceServer velocity_srv = nodeHandle.advertiseService("velocity_service", movementCallback);
    ros::ServiceServer depth_srv = nodeHandle.advertiseService("depth_service", depthCallback);
    ros::ServiceServer dropper_srv = nodeHandle.advertiseService("dropper_service", dropperCallback);
    ros::ServiceServer lifter_srv = nodeHandle.advertiseService("lifter_service", lifterCallback);
    ros::ServiceServer imu_init_srv = nodeHandle.advertiseService("imu_init_service", imuInitCallback);
    ros::ServiceServer stabilization_srv = nodeHandle.advertiseService("stabilization_service", stabilizationCallback);

    std_msgs::UInt32 depthMessage;
    ros::Rate rate(1.0);

    while (ros::ok()) {
        depthMessage.data = currentDepth;
        depthPublisher.publish(depthMessage);
        velocityPublisher.publish(currentTwist);
        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}


bool movementCallback(auv_common::VelocityCmd::Request& velocityRequest,
                       auv_common::VelocityCmd::Response& velocityResponse) {

    currentTwist = velocityRequest.twist;
    velocityResponse.success.data = true;

    return true;
}

bool depthCallback(auv_common::DepthCmd::Request& depthRequest,
                   auv_common::DepthCmd::Response& depthResponse) {
    /* Currently not supported in simulation */
    depthResponse.success.data = true;
    currentDepth = depthRequest.depth;
    return true;
}

bool dropperCallback(auv_common::DropperCmd::Request& dropperRequest,
                      auv_common::DropperCmd::Response& dropperResponse) {

    dropperResponse.success = true;
    return true;
}

bool lifterCallback(auv_common::LifterCmd::Request& lifterRequest,
                     auv_common::LifterCmd::Response& lifterResponse) {

    // TODO: Implement
    return true;
}

bool imuInitCallback(auv_common::EnablingCmd::Request& enablingRequest,
                       auv_common::EnablingCmd::Response& enablingResponse) {

    enablingResponse.success = true;
    return true;
}

bool stabilizationCallback(auv_common::EnablingCmd::Request& enablingRequest,
                            auv_common::EnablingCmd::Response& enablingResponse) {

    enablingResponse.success = true;
    return true;
}
