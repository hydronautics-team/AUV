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

#include <sstream>
#include <string>
#include <vector>

#include "serial.h"
#include "messages.h"

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

}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void movement_callback(const geometry_msgs::Twist::ConstPtr &input)
{
  	request.roll	= static_cast<int16_t> (input->angular.x);
  	request.yaw		= static_cast<int16_t> (input->angular.y);
  	request.pitch	= static_cast<int16_t> (input->angular.z);

  	request.march	= static_cast<int16_t> (input->linear.x);
  	request.depth	= static_cast<int16_t> (input->linear.y);
  	request.lag	    = static_cast<int16_t> (input->linear.z); 

  	isReady = true;
}

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
    
    // ROS publishers
    ros::Publisher outputMessage_pub 	= n.advertise<std_msgs::UInt8MultiArray>("/hard_bridge/parcel", 1000);
    //ros::Publisher outputMessage_pub 	= n.advertise<std_msgs::UInt8MultiArray>("", 1000);
	// **************

    // ROS subscribers
    ros::Subscriber inputMessage_sub 	= n.subscribe("/hard_bridge/uart", 1000, inputMessage_callback);
    ros::Subscriber movement_sub 		= n.subscribe("/pilot/velocity", 1000, movement_callback);
    // **************

    while (ros::ok())
    {        
        if(isReady) {
        	makeOutputMessage();
        	outputMessage_pub.publish(msg_out);
        }

    	ros::spinOnce();
    	communication_delay.sleep();          
    }

    return 0;
}
