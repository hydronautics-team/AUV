#include "ros/ros.h"
#include "serial/serial.h"
#include <iostream>
#include <string>

// TODO: Remove this node

static const std::string UART_BRIDGE_NODE_NAME = "uart_bridge_ros";

static const std::string PARAM_DEVICE = "device";

static const std::string PARAM_BAUDRATE = "baudrate";

static const std::string PARAM_DATA_BYTES = "dataBytes";

static const std::string PARAM_PARITY = "parity";

static const std::string PARAM_STOP_BITS = "stopBits";


static const std::string PARITY_NONE = "none";

static const std::string PARITY_EVEN = "even";

static const std::string PARITY_ODD = "odd";


static const std::string DEFAULT_DEVICE = "/dev/ttyUSB0"; // TODO: Change to /dev/ttyS0

static const int DEFAULT_BAUDRATE = 57600;

static const int DEFAULT_DATA_BYTES = 8;

static const std::string DEFAULT_PARITY = PARITY_NONE;

static const int DEFAULT_STOP_BITS = 1;


static const int DEFAULT_SERIAL_TIMEOUT = 1000;


/**
 * Closes port if it is closed, initialized it
 * with given parameter and DOES NOT OPEN IT.
 */
void initPort(serial::Serial& port, std::string device,
        int baudrate, int timeout, serial::bytesize_t dataBytes, serial::parity_t parity, serial::stopbits_t stopBits) {

    if (port.isOpen())
        port.close();

    port.setPort(device);
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(timeout);
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, UART_BRIDGE_NODE_NAME);
    ros::NodeHandle nodeHandle(UART_BRIDGE_NODE_NAME);

    std::string device;
    nodeHandle.param(PARAM_DEVICE, device, DEFAULT_DEVICE);

    int baudrate;
    nodeHandle.param(PARAM_BAUDRATE, baudrate, DEFAULT_BAUDRATE);

    int dataBytesInt;
    nodeHandle.param(PARAM_DATA_BYTES, dataBytesInt, DEFAULT_DATA_BYTES);
    serial::bytesize_t dataBytes;
    switch (dataBytesInt) {
        case 5:
            dataBytes = serial::bytesize_t::fivebits;
            break;
        case 6:
            dataBytes = serial::bytesize_t::sixbits;
            break;
        case 7:
            dataBytes = serial::bytesize_t::sevenbits;
            break;
        case 8:
            dataBytes = serial::bytesize_t::eightbits;
            break;
        default:
            ROS_INFO("Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
            return 0;
    }

    std::string parityStr;
    nodeHandle.param(PARAM_PARITY, parityStr, DEFAULT_PARITY);
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == PARITY_EVEN)
        parity = serial::parity_t::parity_even;
    else if (parityStr == PARITY_ODD)
        parity = serial::parity_t::parity_odd;
    else if (parityStr == PARITY_NONE)
        parity = serial::parity_t::parity_none;
    else {
        ROS_ERROR("Urecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"", parityStr.c_str());
        return 0;
    }

    int stopBitsInt;
    nodeHandle.param(PARAM_STOP_BITS, stopBitsInt, DEFAULT_STOP_BITS);
    serial::stopbits_t stopBits;
    switch (stopBitsInt) {
        case 1:
            stopBits = serial::stopbits_t::stopbits_one;
            break;
        case 2:
            stopBits = serial::stopbits_t::stopbits_two;
            break;
        default:
            ROS_ERROR("Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
            return 0;
    }



    ROS_INFO("UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
            device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);

    serial::Serial port;
    initPort(port, device, baudrate, DEFAULT_SERIAL_TIMEOUT, dataBytes, parity, stopBits);
    port.open();
    if (!port.isOpen()) {
        ROS_ERROR("Unable to open port, shutting down node");
        return 0;
    }

    // Just for temporary debug
    ros::Rate rate(1.0);
    std::string testStr = "Hello, World!";
    while (ros::ok()) {

        size_t bytes_wrote = port.write(testStr);
        std::string result = port.read(testStr.length() + 1);

        std::cout << "Bytes wrote: " << bytes_wrote << " " << result << std::endl;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}