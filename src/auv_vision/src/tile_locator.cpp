#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <util/ImgprocUtil.h>
#include <common/AbstractImageConverter.h>
#include "std_msgs/Empty.h"


// JUST FOR DEBUG! REAL TOPIC IS /cam_bottom/image_raw
static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string TILE_ANGLE_PUBLISH_TOPIC = "/tile/angle";

static const std::string TILE_POSITION_PUBLISH_TOPIC = "/tile/position";

static const std::string TILE_LOCATOR_NODE_NAME = "tile_locator";

class TilePublisher : public AbstractImageConverter
{

private:

    ros::Publisher anglePublisher;
    ros::Publisher positionPublisher;

    ros::Subscriber resetSubscriber;

    // Stub values
    float angle = -90.0f;
    int positionX = 0;
    int positionY = 0;

protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        if (angle > 90.0f)
            angle = -90.0f;
        std_msgs::Float32 angleMsg;
        angleMsg.data = angle;
        anglePublisher.publish(angleMsg);

        if (positionX > 2000)
            positionX = 0;
        if (positionY > 2000)
            positionY = 0;
        positionX++;
        positionY++;
        geometry_msgs::Point positionMsg;
        positionMsg.x = positionX;
        positionMsg.y = positionY;
        positionMsg.z = 0;
        positionPublisher.publish(positionMsg);
    }

public:

    TilePublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        anglePublisher = nodeHandle.advertise<std_msgs::Float32>(TILE_ANGLE_PUBLISH_TOPIC, 100);
        positionPublisher = nodeHandle.advertise<geometry_msgs::Point>(TILE_POSITION_PUBLISH_TOPIC, 100);
    }

    ~TilePublisher()
    {
    }

};


/* TODO Fix code style */
int main(int argc, char **argv)
{

    ros::init(argc, argv, TILE_LOCATOR_NODE_NAME);
    TilePublisher tilePublisher(CAMERA_TOPIC);

    ros::spin();

    return 0;
}