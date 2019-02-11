#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <util/ImgprocUtil.h>
#include <common/AbstractImageConverter.h>
#include "std_msgs/Empty.h"
#include "../include/tile/TileAngle.h"

// JUST FOR DEBUG! REAL TOPIC IS /cam_bottom/image_raw
static const std::string OPENCV_WINDOW = "tile_window";

static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string TILE_GLOBAL_ANGLE_PUBLISH_TOPIC = "/tile/angle/global";

static const std::string TILE_VERTICAL_ANGLE_PUBLISH_TOPIC = "/tile/angle/vertical";

static const std::string TILE_POSITION_PUBLISH_TOPIC = "/tile/position";

static const std::string TILE_LOCATOR_NODE_NAME = "tile_locator";

class TilePublisher : public AbstractImageConverter
{

private:

    ros::Publisher globalAnglePublisher;
    ros::Publisher verticalAnglePublisher;
    ros::Publisher positionPublisher;

    ros::Subscriber resetSubscriber;

    // Stub values
    int positionX = 0;
    int positionY = 0;

    TileAngle tracker;

protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        cv::Mat frame = cv_ptr->image;
        std_msgs::Float32 globalAngle, verticalAngle;

        tracker.processing(frame);
        cv::imshow(OPENCV_WINDOW, frame);
        cv::waitKey(3);

        globalAngle.data = tracker.globalAngle();
        verticalAngle.data = tracker.verticalAngle();

        globalAnglePublisher.publish(globalAngle);
        verticalAnglePublisher.publish(verticalAngle);

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
        cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);

        globalAnglePublisher = nodeHandle.advertise<std_msgs::Float32>(TILE_GLOBAL_ANGLE_PUBLISH_TOPIC, 100);
        verticalAnglePublisher = nodeHandle.advertise<std_msgs::Float32>(TILE_VERTICAL_ANGLE_PUBLISH_TOPIC, 100);
        positionPublisher = nodeHandle.advertise<geometry_msgs::Point>(TILE_POSITION_PUBLISH_TOPIC, 100);
    }

    ~TilePublisher()
    {
        cv::destroyWindow(OPENCV_WINDOW);
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