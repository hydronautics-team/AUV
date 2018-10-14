/**
 * This node:
 * - receives byte data from stm by i2c, converts it to byte array and sends
 *   it to message_handler;
 * - receives byte array to send from message_handler and sends to stm by i2c;
 */

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include "messages.h" // TODO maybe this isn't necessary
#include <typeinfo> // TODO remove
#include <stdint.h>

std_msgs::UInt8MultiArray msg_in;
std_msgs::UInt8MultiArray msg_to_send;

/**
 * Jetson -> STM
 */
bool sendData(){
	// TODO send data to i2c
	// uint8_t[] data = msg_to_send.data; // example of unpacking data from msg
	return false;
}

/**
 * STM -> Jetson
 */
bool receiveData(){
	
	// TODO receive data from i2c
	// msg_in[0] = 42; // - example of writing uint8_t to array
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "i2c_server");
	ros::NodeHandle n;

	ros::Publisher advertise_msg_in = n.advertise<std_msgs::UInt8MultiArray>("msg_in", 2);
	ros::Rate delay_request_responce(1); // TODO make both as param
	ros::Rate delay_responce_request(1);

	std_msgs::MultiArrayDimension myDim;                            //MultiArrayDimension structure is defined
	std_msgs::MultiArrayLayout myLayout; 

	msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg_in.layout.dim[0].size = REQUEST_NORMAL_LENGTH;
	msg_in.layout.dim[0].stride = REQUEST_NORMAL_LENGTH;
	msg_in.layout.dim[0].label = "x";
	msg_in.layout.data_offset = 0;

	// TODO make this work
	// uint8_t* x = (uint8_t *)malloc(sizeof(uint8_t)*REQUEST_NORMAL_LENGTH);
	// msg_in.data = (uint8_t *)malloc(sizeof(uint8_t)*REQUEST_NORMAL_LENGTH); // TODO fix possible memory leak

	// TODO make the same with msg_to_send

  	while (ros::ok())
  	{
	  	// ---===### INITIALIZING CYCLE STEP ###===---


	    // ---===### SENDING MESSAGE ###===---
	    // TODO get msg_in from publisher
	    if(!sendData()){
	    	ROS_INFO("Unable to send msg to stm");
	    }
	    delay_request_responce.sleep();


	    // ---===### RECEIVING MESSAGE ###===---
	    if(!receiveData()){
	    	ROS_INFO("Unable to receive msg from stm");
	    }
	    advertise_msg_in.publish(msg_in);
	    delay_responce_request.sleep();


	    // ---===### FINALIZING CYCLE STEP ###===---
	    ros::spinOnce();
  	}
  	return 0;
}
