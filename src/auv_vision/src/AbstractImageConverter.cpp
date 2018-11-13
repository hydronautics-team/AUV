#include "AbstractImageConverter.h"

AbstractImageConverter::AbstractImageConverter(const std::string& inputImageTopic)
    : imageTransport(nodeHandle)
{
    imageSubscriber = imageTransport.subscribe(inputImageTopic, 1, &AbstractImageConverter::imageCb, this);
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

