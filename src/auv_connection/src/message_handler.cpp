#include "ros/ros.h"
#include "messages.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8MultiArray.h"

#include "geometry_msgs/Twist.h"

#include "sensor_msgs/Imu.h"


#include <sstream>


#define MESSAGES_ARRAY_SIZE REQUEST_CONFIG_LENGTH


// Jetson -> STM
std_msgs::UInt8MultiArray msg_to_send;
geometry_msgs::Twist movement_msg;

// STM -> Jetson
std_msgs::UInt8MultiArray msg_in;
sensor_msgs::Imu imu;
std_msgs::UInt16 pressure;



uint8_t isCheckSumm16bCorrect(uint8_t * msg, uint16_t length)
{
    uint16_t crcGot, crc = 0;
    int i;

    crcGot = (uint16_t)( msg[length-1] + (msg[length-2] << 8) );

    for(i=0; i < length - 2; i++){ //i теперь не с 0, а с 1
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= msg[i];
        crc ^= (uint8_t)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    if(crc == crcGot )
        return 1;
    else return 0;
}

void addCheckSumm16b(uint8_t * msg, uint16_t length)//i теперь не с 0, а с 1
{
    uint16_t crc = 0;
    int i = 0;

    for(i=0; i < length - 2; i++){
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= msg[i];
        crc ^= (uint8_t)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    msg[length-2] = (uint8_t) (crc >> 8);
    msg[length-1] = (uint8_t) crc;
}

void makeMsgOut()
{
  
  uint8_t byte_array[MESSAGES_ARRAY_SIZE];
  int16_t pitch       = (static_cast<int16_t> (movement_msg.angular.y));     //[-32767,32767] (вперед)
  int16_t roll        = (static_cast<int16_t> (movement_msg.angular.x));     //[-32767,32767] (право)
  int16_t yaw         = (static_cast<int16_t> (movement_msg.angular.z));     //[-32767,32767] (вправо)

  int16_t march       = (static_cast<int16_t> (movement_msg.linear.x));     //[-32767,32767] (погружение)
  int16_t lag         = (static_cast<int16_t> (movement_msg.linear.y));     //[-32767,32767] (против ч/ч)
  int16_t depth       = (static_cast<int16_t> (movement_msg.linear.z)); 



  for (int i = 0; i < REQUEST_NORMAL_LENGTH; ++i) {
      byte_array[i] = 0x00;
  }

  byte_array[REQUEST_NORMAL_TYPE] = REQUEST_NORMAL_CODE;

  byte_array[REQUEST_NORMAL_MARCH+1] = march;
  byte_array[REQUEST_NORMAL_MARCH] = march >> 8;

  byte_array[REQUEST_NORMAL_LAG+1] = lag;
  byte_array[REQUEST_NORMAL_LAG] = lag >> 8;

  byte_array[REQUEST_NORMAL_DEPTH+1] = depth;
  byte_array[REQUEST_NORMAL_DEPTH] = depth >> 8;

  byte_array[REQUEST_NORMAL_ROLL+1] = roll;
  byte_array[REQUEST_NORMAL_ROLL] = roll >> 8;

  byte_array[REQUEST_NORMAL_PITCH+1] = pitch;
  byte_array[REQUEST_NORMAL_PITCH] = pitch >> 8;

  byte_array[REQUEST_NORMAL_YAW+1] = yaw;
  byte_array[REQUEST_NORMAL_YAW] = yaw >> 8;

  addCheckSumm16b(byte_array, REQUEST_NORMAL_LENGTH);

  for (int i = 0; i < REQUEST_NORMAL_LENGTH; ++i)
  {
    msg_to_send.data[i] = byte_array[i];
  }
  
  //newPort->write((char*)msg_to_send, REQUEST_NORMAL_LENGTH);
}


void msgIn(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void movement(const geometry_msgs::Twist::ConstPtr& msg)
{
  movement_msg.linear.x = msg->linear.x;
  movement_msg.linear.y = msg->linear.y;
  movement_msg.linear.z = msg->linear.z;

  movement_msg.angular.x = msg->angular.x;
  movement_msg.angular.y = msg->angular.y;
  movement_msg.angular.z = msg->angular.z;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "message_handler");

  

  //msg_to_send.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension)*2);

  //msg_to_send.layout.dim[0].size = MESSAGES_ARRAY_SIZE;

  msg_to_send.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg_to_send.layout.dim[0].size = REQUEST_NORMAL_LENGTH;
  msg_to_send.layout.dim[0].stride = 1;
  msg_to_send.layout.dim[0].label = "x";


  //msg_to_send.data = new uint8_t[8];//(uint8_t *)malloc(sizeof(uint8_t)*MESSAGES_ARRAY_SIZE);

  ros::NodeHandle n;

  
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("msg_in_imu", 1000);
  ros::Publisher pressure_pub = n.advertise<std_msgs::UInt16>("msg_in_pressure", 1000);
  ros::Publisher msg_out_pub = n.advertise<std_msgs::UInt8MultiArray>("msg_out", 1000);


  ros::Subscriber movement_sub = n.subscribe("movement", 1000, movement);
  ros::Subscriber msg_in_sub = n.subscribe("msg_in", 1000, msgIn);


  ros::Rate loop_rate(1000);

  
  pressure.data = 0;

  while (ros::ok())
  {
    pressure.data = pressure.data + 1;

    // ROS_INFO("%s", msg.data.c_str());

    pressure_pub.publish(pressure);

    ros::spinOnce();

    loop_rate.sleep();

    makeMsgOut();
    msg_out_pub.publish(msg_to_send);
  }

  // ros::spin(); // ? Было в subscriber

  return 0;
}