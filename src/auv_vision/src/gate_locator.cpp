#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include "gate/AngleGateDetector.h"
#include "gate/VerticalGateDetector.h"
#include <util/ImgprocUtil.h>
#include <dynamic_reconfigure/server.h>
#include <auv_vision/GateLocatorConfig.h>
#include "common/AbstractImageConverter.h"
#include "auv_common/OptionalPoint2D.h"
#include "auv_common/Gate.h"

static const std::string OPENCV_WINDOW = "Image window";

static const std::string ENABLE_WINDOWS_PARAM = "debugVision";

static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string GATE_PUBLISH_TOPIC = "/gate";

static const std::string GATE_LOCATOR_NODE_NAME = "gate_locator";

static const std::string DETECTOR_MODE_PARAM = "detectorMode";

static const std::string DETECTOR_MODE_VERTICAL_STR = "vertical";

static const std::string DETECTOR_MODE_ANGLE_STR = "angle";

static const std::string HORIZONTAL_TO_VERTICAL_RELATION_PARAM = "horizontalToVerticalRelation";

typedef enum {
    VERTICAL,
    ANGLE
} DetectorMode;


class GatePublisher : public AbstractImageConverter {

private:

    IGateDetector* detector;

    dynamic_reconfigure::Server<auv_vision::GateLocatorConfig> reconfigurationServer;

    image_transport::Publisher imagePublisher;

    ros::Publisher gatePublisher;

    bool windowsEnabled;
    
protected:

    void process(const cv_bridge::CvImagePtr &cv_ptr) override {
        cv::Mat image = cv_ptr->image;

        GateDescriptor gate = detector->detect(image);

        auv_common::Gate msg;

        if (gate.hasGate()) {
            cv::Point2f center = convertToCentralCoordinates(gate.getCenter(), image.cols, image.rows);
            std::vector<cv::Point2f> corners = gate.getCorners();

            msg.isPresent = true;
            msg.xCenter = center.x;
            msg.yCenter = center.y;
            msg.xTL = corners[0].x;
            msg.yTL = corners[0].y;
            msg.xTR = corners[1].x;
            msg.yTR = corners[1].y;
            msg.xBR = corners[2].x;
            msg.yBR = corners[2].y;
            msg.xBL = corners[3].x;
            msg.yBL = corners[3].y;
            gatePublisher.publish(msg);

            cv::circle(image, corners[0], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[1], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[2], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[3], 10, CV_RGB(0, 255, 0));
            cv::circle(image, gate.getCenter(), 10, CV_RGB(0, 0, 255));
        } else {
            msg.isPresent = false;
            msg.xCenter = 0.0f;
            msg.yCenter = 0.0f;
            msg.xTL = 0.0f;
            msg.yTL = 0.0f;
            msg.xTR = 0.0f;
            msg.yTR = 0.0f;
            msg.xBR = 0.0f;
            msg.yBR = 0.0f;
            msg.xBL = 0.0f;
            msg.yBL = 0.0f;
            gatePublisher.publish(msg);
        }

        if (windowsEnabled) {
            cv::imshow(OPENCV_WINDOW, image);
            cv::waitKey(3);
        }

        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        imagePublisher.publish(imageMsg);
    }

    void reconfigure(auv_vision::GateLocatorConfig& config, uint32_t level) {
        detector->reconfigure(config, level);
    }


public:

    GatePublisher(const std::string &inputImageTopic, const DetectorMode& detectorMode,
            bool enableWindows, float horizontalToVerticalRelation, const ros::NodeHandle& nh) : AbstractImageConverter(inputImageTopic),
                                                                windowsEnabled(enableWindows) {

        if (detectorMode == DetectorMode::ANGLE)
            detector = new AngleGateDetector(horizontalToVerticalRelation);
        else
            detector = new VerticalGateDetector(horizontalToVerticalRelation);
        detector->setPublisher(nh);

        dynamic_reconfigure::Server<auv_vision::GateLocatorConfig>::CallbackType f;
        f = boost::bind(&GatePublisher::reconfigure, this, _1, _2);
        reconfigurationServer.setCallback(f);

        image_transport::ImageTransport it(nodeHandle);
        imagePublisher = it.advertise("/gate/image", 100);

        gatePublisher = nodeHandle.advertise<auv_common::Gate>(GATE_PUBLISH_TOPIC, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
    }

    ~GatePublisher() {
        delete detector;
        if (windowsEnabled)
            cv::destroyWindow(OPENCV_WINDOW);
    }

};


/* TODO Fix code style */
int main(int argc, char **argv)
{

#if CV_MAJOR_VERSION == 2
    ROS_INFO("%s", "OpenCV version = 2");
#elif CV_MAJOR_VERSION == 3
    ROS_INFO("%s", "OpenCV version = 3");
#endif

    ros::init(argc, argv, GATE_LOCATOR_NODE_NAME);

    ros::NodeHandle nodeHandle(GATE_LOCATOR_NODE_NAME);

    bool windowsEnabled;
    nodeHandle.param(ENABLE_WINDOWS_PARAM, windowsEnabled, false);
    std::string detectorModeStr;
    nodeHandle.param(DETECTOR_MODE_PARAM, detectorModeStr, DETECTOR_MODE_VERTICAL_STR);
    DetectorMode detectorMode;
    if (detectorModeStr == DETECTOR_MODE_VERTICAL_STR)
        detectorMode = DetectorMode::VERTICAL;
    else if (detectorModeStr == DETECTOR_MODE_ANGLE_STR)
        detectorMode = DetectorMode::ANGLE;
    double horizontalToVerticalRelation;
    nodeHandle.param(HORIZONTAL_TO_VERTICAL_RELATION_PARAM, horizontalToVerticalRelation, 1.5);

    GatePublisher gatePublisher(CAMERA_TOPIC, detectorMode, windowsEnabled, horizontalToVerticalRelation, nodeHandle);

    ros::spin();

    return 0;
}
