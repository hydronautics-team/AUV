#ifndef AUV_VISION_ABSTRACTIMAGECONVERTER_H
#define AUV_VISION_ABSTRACTIMAGECONVERTER_H

#include "ros/ros.h"
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>


/**
 * <br>Base class that helps working with cv_bridge.
 * Subscribes on "image_raw" topic and applies
 * to it method <b><code>process</code></b> that
 * is implemented in child class.
 * <br>
 * <br>Note that base class does not publish
 * data to any topics, all publishing logic
 * has to be implemented in child classes.
 */
class AbstractImageConverter
{

protected:

    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    /* Image processing logic, should be overridden in child classes */
    virtual void process(const cv_bridge::CvImagePtr& cv_ptr) = 0;

public:

    AbstractImageConverter(const std::string& inputImageTopic);
    ~AbstractImageConverter() = default;

};

#endif //AUV_VISION_ABSTRACTIMAGECONVERTER_H
