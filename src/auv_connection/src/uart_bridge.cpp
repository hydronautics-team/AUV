/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
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

// Name of the device file 
std::string file = "/dev/ttyUSB0";

// Hardware bridge -> Protocol_bridge
std_msgs::UInt8MultiArray msg_in;

// Protocol_bridge -> Hardware bridge
std_msgs::UInt8MultiArray msg_out;

bool isMessageReceived = false;

static uint64_t GetTickCountMs();

/// Returns the number of ticks since an undefined time (usually system startup).
static uint64_t GetTickCountMs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_nsec / 1000000) + (static_cast<uint64_t>(ts.tv_sec) * 1000ull);
}

bool sendData(Serial &port)
{
	std::vector<uint8_t> msg;
	for(int i=0; i<RequestMessage::length; i++) {
		msg.push_back(msg_in.data[i]);
	}
	port << msg;
	return true;
}

bool receiveData(Serial &port)
{
    uint64_t lasttick = GetTickCountMs();
    if(port.bytesAvailable() < ResponseMessage::length) {
   		std::vector<uint8_t> answer;
   		port >> answer;

   		for(int i=0; i<ResponseMessage::length; i++) {
			msg_out.data.push_back(answer[i]);
		}

		return true;
   	}
   	else {
   		return false;
   	}
}

/** @brief Parse string bitwise correctly into ResponseMessage and check 16bit checksum.
  *
  * @param[in]  &input String to parse.
  */
void inputMessage_callback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
	for(int i=0; i<RequestMessage::length; i++) {
		msg_in.data.push_back(msg->data[i]);
	}
	isMessageReceived = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uart_bridge");
	ros::NodeHandle n;

	ros::Rate delay_request_responce(1);
	ros::Rate delay_responce_request(1);

    // Input message container
    msg_in.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_in.layout.dim[0].size = RequestMessage::length;
    msg_in.layout.dim[0].stride = RequestMessage::length;
    msg_in.layout.dim[0].label = "msg_in";

    // Outnput message container
    msg_out.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_out.layout.dim[0].size = ResponseMessage::length;
    msg_out.layout.dim[0].stride = ResponseMessage::length;
    msg_out.layout.dim[0].label = "msg_out";

    // ROS publishers
    ros::Publisher outputMessage_pub 	= n.advertise<std_msgs::UInt8MultiArray>("/hard_bridge/uart", 1000);
	// **************

    // ROS subscribers
    ros::Subscriber inputMessage_sub 	= n.subscribe("/hard_bridge/parcel", 1000, inputMessage_callback);
    // **************

    // Initialasing serial port
    Serial port(file, 57600, 8, PARITY_NONE, 1);

  	while (ros::ok())
  	{
  		if(!isMessageReceived) {
  			continue;
  		}

	    if(!sendData(port)) {
	    	ROS_INFO("Unable to send msg to STM32");
	    }
	    delay_request_responce.sleep();

	    if(receiveData(port)) {
	    	outputMessage_pub.publish(msg_in);
	    }
	    else {
	   		ROS_INFO("Unable to receive msg from STM32");
	    }
	    delay_responce_request.sleep();

	    ros::spinOnce();
  	}
  	return 0;
}
