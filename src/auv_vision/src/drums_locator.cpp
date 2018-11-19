#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <gate/GateDescriptor.h>
#include <gate/GateDetector.h>
#include <util/ImgprocUtil.h>
#include <common/AbstractImageConverter.h>
#include "auv_common/LabeledPoint2D.h"


// JUST FOR DEBUG! REAL TOPIC IS /cam_bottom/image_raw
static const std::string CAMERA_BOTTOM_TOPIC = "/cam_front_1/image_raw";

static const std::string CAMERA_FRONT_TOPIC = "/cam_front_1/image_raw";

static const std::string DRUMS_MAT_PUBLISH_TOPIC = "/drums/mat";

static const std::string DRUMS_DRUM_PUBLISH_TOPIC = "/drums/drum";

static const std::string DRUMS_LOCATOR_NODE_NAME = "drums_locator";

class DrumPublisher : public AbstractImageConverter
{

private:

    ros::Publisher publisher;


protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        auv_common::LabeledPoint2D msg;
        // Stub values
        msg.x = 10.0;
        msg.y = 15.0;
        msg.label = 1;
        publisher.publish(msg);
    }

public:

    DrumPublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        publisher = nodeHandle.advertise<auv_common::LabeledPoint2D>(DRUMS_DRUM_PUBLISH_TOPIC, 100);
    }

    ~DrumPublisher()
    {
    }

};

class MatPublisher : public AbstractImageConverter
{

private:

    ros::Publisher publisher;


protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        auv_common::LabeledPoint2D msg;
        // Stub values
        msg.x = 20.0f;
        msg.y = 25.0f;
        msg.label = 3;
        publisher.publish(msg);
    }

public:

    MatPublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        publisher = nodeHandle.advertise<auv_common::LabeledPoint2D>(DRUMS_MAT_PUBLISH_TOPIC, 100);
    }

    ~MatPublisher()
    {
    }

};


/* TODO Fix code style */
int main(int argc, char **argv)
{
    ros::init(argc, argv, DRUMS_LOCATOR_NODE_NAME);
    DrumPublisher gatePublisher(CAMERA_BOTTOM_TOPIC);
    MatPublisher matPublisher(CAMERA_FRONT_TOPIC);

    ros::spin();

    return 0;
}