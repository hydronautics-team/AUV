/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include "ros/ros.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

#include <auv_common/VelocityCmd.h>
#include <auv_common/DepthCmd.h>
#include <auv_common/DropperCmd.h>
#include <auv_common/LifterCmd.h>
#include <auv_common/EnablingCmd.h>

#include <sstream>
#include <string>
#include <vector>
#include <std_msgs/UInt32.h>

#include "messages.h"

#define SHORE_STABILIZE_DEPTH_BIT       0
#define SHORE_STABILIZE_ROLL_BIT        1
#define SHORE_STABILIZE_PITCH_BIT       2
#define SHORE_STABILIZE_YAW_BIT         3
#define SHORE_STABILIZE_IMU_BIT         4
#define SHORE_STABILIZE_SAVE_BIT		5

// High level -> Hardware bridge
geometry_msgs::Twist movement;

// Hardware bridge -> High level
sensor_msgs::Imu telemetry;

// Hardware bridge -> Protocol_bridge
std_msgs::UInt8MultiArray msg_out;

// Protocol_bridge -> Hardware bridge
std_msgs::UInt8MultiArray msg_in;

const uint32_t communicationDelayMilliseconds = 100;
bool isReady = false;

RequestMessage request;
ResponseMessage response;

int current_depth = 0;

bool pick_bit(uint8_t &input, uint8_t bit)
{
    return static_cast<bool>((input << (8 - bit)) >> 8);
}


void set_bit(uint8_t &byte, uint8_t bit, bool state)
{
    uint8_t value = 1;
    if(state) {
        byte = byte | (value << bit);
    }
    else {
        byte = byte & ~(value << bit);
    }
}

/** @brief Make byte array to publish for protocol_node
  *
  */
void makeOutputMessage()
{
	std::vector<uint8_t> output_vector = request.formVector();

  	msg_out.data.clear();
  	for(int i=0; i<RequestMessage::length; i++) {
  	  msg_out.data.push_back(output_vector[i]);
  	}
}

/** @brief Parse input byte array from protocol_node subscriber
  *
  */
void parseInputMessage()
{

}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void inputMessage_callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    std::vector<uint8_t> received_vector;
    for(int i=0; i<ResponseMessage::length; i++) {
        received_vector.push_back(msg->data[i]);
    }
    bool ok = response.parseVector(received_vector);
    if (ok)
        current_depth = std::abs((int)response.depth) * 100; // Convert metres to centimetres
    else
        ROS_ERROR("Wrong checksum");
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
bool movement_callback(auv_common::VelocityCmd::Request& velocityRequest,
        auv_common::VelocityCmd::Response& velocityResponse)
{
  	request.roll	= static_cast<int16_t> (velocityRequest.twist.angular.x);
  	request.yaw		= static_cast<int16_t> (velocityRequest.twist.angular.y);
  	request.pitch	= static_cast<int16_t> (velocityRequest.twist.angular.z);

  	request.march	= static_cast<int16_t> (velocityRequest.twist.linear.x);
  	request.lag	    = static_cast<int16_t> (velocityRequest.twist.linear.z);

    isReady = true;

  	velocityResponse.success.data = true;

    return true;
}

bool depth_callback(auv_common::DepthCmd::Request& depthRequest,
                       auv_common::DepthCmd::Response& depthResponse)
{
    ROS_INFO("Setting depth to %d", depthRequest.depth);
    request.depth	= -(static_cast<int16_t> (depthRequest.depth * 10)); // For low-level stabilization purposes
    ROS_INFO("Sending to STM32 depth value: %d", request.depth);

    isReady = true;

    depthResponse.success.data = true;

    return true;
}

bool dropper_callback(auv_common::DropperCmd::Request& dropperRequest,
                    auv_common::DropperCmd::Response& dropperResponse) {

    ROS_INFO("Dropper velocity: %d", dropperRequest.velocity);
    request.dev[1] = dropperRequest.velocity;

    isReady = true;

    return true;
}

bool lifter_callback(auv_common::LifterCmd::Request& lifterRequest,
                      auv_common::LifterCmd::Response& lifterResponse) {

    // TODO: Implement
    return true;
}

bool imu_init_callback(auv_common::EnablingCmd::Request& enablingRequest,
                        auv_common::EnablingCmd::Response& enablingResponse) {

    ROS_INFO("Setting SHORE_STABILIZE_IMU_BIT to %d", enablingRequest.enabled);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_IMU_BIT, enablingRequest.enabled);

    isReady = true;

    enablingResponse.success = true;

    return true;
}

bool stabilization_callback(auv_common::EnablingCmd::Request& enablingRequest,
                       auv_common::EnablingCmd::Response& enablingResponse) {

    ROS_INFO("Setting stabilization bits to %d", enablingRequest.enabled);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT, enablingRequest.enabled);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_YAW_BIT, enablingRequest.enabled);

    isReady = true;

    enablingResponse.success = true;

    return true;
}

//bool imu_init_callback()

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_bridge");
    ros::NodeHandle n;

    ros::Rate communication_delay(1000/communicationDelayMilliseconds);

    // Input message container
    msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_in.layout.dim[0].size = ResponseMessage::length;
    msg_in.layout.dim[0].stride = ResponseMessage::length;
    msg_in.layout.dim[0].label = "msg_in";

    // Outnput message container
    msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_out.layout.dim[0].size = RequestMessage::length;
    msg_out.layout.dim[0].stride = RequestMessage::length;
    msg_out.layout.dim[0].label = "msg_out";

    request.roll = 0;
    request.yaw	= 0;
    request.pitch = 0;
    request.depth = 0;
    request.march = 0;
    request.lag	= 0;
    set_bit(request.stabilize_flags, SHORE_STABILIZE_DEPTH_BIT, false);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_YAW_BIT, false);
    set_bit(request.stabilize_flags, SHORE_STABILIZE_IMU_BIT, false);

    std_msgs::UInt32 depth_message;
    current_depth = 50;
    
    // ROS publishers
    ros::Publisher outputMessage_pub 	= n.advertise<std_msgs::UInt8MultiArray>("/hard_bridge/parcel", 1000);
    ros::Publisher depth_pub = n.advertise<std_msgs::UInt32>("/perception/depth", 1000);
    //ros::Publisher outputMessage_pub 	= n.advertise<std_msgs::UInt8MultiArray>("", 1000);
	// **************

    // ROS subscribers
    ros::Subscriber inputMessage_sub 	= n.subscribe("/hard_bridge/uart", 1000, inputMessage_callback);
    // **************

    // ROS services
    ros::ServiceServer velocity_srv = n.advertiseService("velocity_service", movement_callback);
    ros::ServiceServer depth_srv = n.advertiseService("depth_service", depth_callback);
    ros::ServiceServer dropper_srv = n.advertiseService("dropper_service", dropper_callback);
    ros::ServiceServer lifter_srv = n.advertiseService("lifter_service", lifter_callback);
    ros::ServiceServer imu_init_srv = n.advertiseService("imu_init_service", imu_init_callback);
    ros::ServiceServer stabilization_srv = n.advertiseService("stabilization_service", stabilization_callback);
    // **************

    while (ros::ok())
    {        
        if(isReady) {
        	makeOutputMessage();
        	outputMessage_pub.publish(msg_out);

        	depth_message.data = current_depth;
        	depth_pub.publish(depth_message);
        }

    	ros::spinOnce();
    	communication_delay.sleep();          
    }

    return 0;
}
