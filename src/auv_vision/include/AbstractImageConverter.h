#ifndef AUV_VISION_ABSTRACTIMAGECONVERTER_H
#define AUV_VISION_ABSTRACTIMAGECONVERTER_H

#include "ros/ros.h"
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>



/**
 * <br>Base class that helps working with cv_bridge.
 * Subscribes on "image_raw" topic and applies
 * to it method <b><code>process</code></b> that
 * is implemented in child class.
 * <br>
 * <br>Note that base class does not publish
 * data to any topics, all publishing logic
 * has to be implmented in child classes.
 */
class AbstractImageConverter
{

protected:

    /* Defines which topic used to retrieve image */
    //const std::string inputImageTopic = "image_raw";
    const std::string inputImageTopic = "/ROV_model_URDF/camera1/image_raw";

    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /* Image processing logic, should be overridden in child classes */
    virtual void process(const cv_bridge::CvImagePtr& cv_ptr) = 0;

public:

    AbstractImageConverter();
    ~AbstractImageConverter() = default;

};

#endif //AUV_VISION_ABSTRACTIMAGECONVERTER_H
