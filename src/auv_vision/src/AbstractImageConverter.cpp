#include "AbstractImageConverter.h"

AbstractImageConverter::AbstractImageConverter()
    : imageTransport(nodeHandle)
{
    // Subscribe to input video feed and publish output video feed
    ROS_INFO("subscribe to input video");
    imageSubscriber = imageTransport.subscribe(inputImageTopic, 1, &AbstractImageConverter::imageCb, this);
    //ROS_INFO("cv_bridge exception: %s");
    //imageSubscriber = imageTransport.subscribe("/mybot/camera1/image_raw", 1,
      //&AbstractImageConverter::imageCb, this);
}

void AbstractImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    process(cv_ptr);
}

