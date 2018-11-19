#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <util/ImgprocUtil.h>
#include <common/AbstractImageConverter.h>
#include "auv_common/OptionalPoint2D.h"


static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string FLARE_PUBLISH_TOPIC = "/flare";

static const std::string FLARE_LOCATOR_NODE_NAME = "flare_locator";

class FlarePublisher : public AbstractImageConverter
{

private:

    ros::Publisher publisher;

protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        auv_common::OptionalPoint2D msg;
        // Stub values
        msg.hasPoint = true;
        msg.x = 10.0f;
        msg.y = 15.0f;
        publisher.publish(msg);
    }

public:

    FlarePublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
      publisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(FLARE_PUBLISH_TOPIC, 100);
    }

    ~FlarePublisher()
    {
    }

};


/* TODO Fix code style */
int main(int argc, char **argv)
{

  ros::init(argc, argv, FLARE_LOCATOR_NODE_NAME);
  FlarePublisher flarePublisher(CAMERA_TOPIC);

  ros::spin();

  return 0;
}