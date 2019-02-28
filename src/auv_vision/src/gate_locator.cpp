#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <gate/GateDescriptor.h>
#include <gate/GateDetector.h>
#include <util/ImgprocUtil.h>
#include <dynamic_reconfigure/server.h>
#include <auv_vision/GateLocatorConfig.h>
#include "common/AbstractImageConverter.h"
#include "auv_common/OptionalPoint2D.h"

static const std::string OPENCV_WINDOW = "Image window";

static const std::string ENABLE_WINDOWS_PARAM = "debugVision";

static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string GATE_PUBLISH_TOPIC = "/gate";

static const std::string GATE_LOCATOR_NODE_NAME = "gate_locator";


GateDetector detector;

void reconfigure(auv_vision::GateLocatorConfig& config, uint32_t level) {
    detector.setVerticalSlope(config.verticalSlope);
    detector.setHorizontalSlope(config.horizontalSlope);
    detector.setLengthRelation(config.lengthRelation);
    detector.setMergingLineDistanceHorizontal(config.mergingLineDistanceHorizontal);
    detector.setMergingLineDistanceVertical(config.mergingLineDistanceVertical);
    detector.setOverlapThreshold(config.overlapThreshold);
    detector.setDistXThreshold(config.distXThreshold);
    detector.setDistYThreshold(config.distYThreshold);
    detector.setSidesRelationThreshold(config.sidesRelationThreshold);
    detector.setAngleDiffThreshold(config.angleDiffThreshold);
    detector.setAreaFrameRelationThreshold(config.areaFrameRelationThreshold);
    detector.setLength_threshold(config.fld_length_threshold);
    detector.setDistance_threshold(config.fld_distance_threshold);
    detector.setCanny_th1(config.fld_canny_threshold_1);
    detector.setCanny_th2(config.fld_canny_threshold_2);
    if (config.fld_canny_aperture == 3.0 || config.fld_canny_aperture == 5.0 || config.fld_canny_aperture == 7.0)
        detector.setCanny_aperture_size(config.fld_canny_aperture);

}

class GatePublisher : public AbstractImageConverter {

private:

    dynamic_reconfigure::Server<auv_vision::GateLocatorConfig> reconfigurationServer;

    image_transport::Publisher imagePublisher;

    ros::Publisher gatePublisher;

    bool windowsEnabled;

protected:

    void process(const cv_bridge::CvImagePtr &cv_ptr) {
        cv::Mat image = cv_ptr->image;

        GateDescriptor gate = detector.detect(image);

        auv_common::OptionalPoint2D msg;

        if (gate.hasGate()) {
            cv::Point2f center = convertToCentralCoordinates(gate.getCenter(), image.cols, image.rows);

            msg.hasPoint = true;
            msg.x = center.x;
            msg.y = center.y;
            gatePublisher.publish(msg);

            std::vector<cv::Point2f> corners = gate.getCorners();
            cv::circle(image, corners[0], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[1], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[2], 10, CV_RGB(0, 255, 0));
            cv::circle(image, corners[3], 10, CV_RGB(0, 255, 0));
            cv::circle(image, gate.getCenter(), 10, CV_RGB(0, 0, 255));
        } else {
            msg.hasPoint = false;
            msg.x = 0;
            msg.y = 0;
            gatePublisher.publish(msg);
        }

        if (windowsEnabled) {
            cv::imshow(OPENCV_WINDOW, image);
            cv::waitKey(3);
        }

        sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        imagePublisher.publish(imageMsg);
    }

public:

    GatePublisher(const std::string &inputImageTopic, bool enableWindows) : AbstractImageConverter(inputImageTopic),
                                                                            windowsEnabled(enableWindows) {

        dynamic_reconfigure::Server<auv_vision::GateLocatorConfig>::CallbackType f;
        f = boost::bind(&reconfigure, _1, _2);
        reconfigurationServer.setCallback(f);

        image_transport::ImageTransport it(nodeHandle);
        imagePublisher = it.advertise("/gate/image", 100);

        gatePublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(GATE_PUBLISH_TOPIC, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
    }

    ~GatePublisher() {
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

    detector.setPublisher(nodeHandle);

    bool windowsEnabled;
    nodeHandle.param(ENABLE_WINDOWS_PARAM, windowsEnabled, false);

    GatePublisher gatePublisher(CAMERA_TOPIC, windowsEnabled);

    ros::spin();

    return 0;
}
