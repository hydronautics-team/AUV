#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <gate/GateDescriptor.h>
#include <gate/GateDetector.h>
#include "../include/AbstractImageConverter.h"
#include "std_msgs/String.h"
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class SampleImageConverter : public AbstractImageConverter
{

private:

    ros::Publisher gatePublisher;

    GateDetector detector;

protected:



    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
      cv::Mat image = cv_ptr->image;

      GateDescriptor gate = detector.detect(image, true);

      if (gate.hasGate()) {
          cv::Point2f center = gate.getCenter();
          geometry_msgs::Pose2D msg;
          msg.x = center.x;
          msg.y = center.y;
          msg.theta = 0;
          gatePublisher.publish(msg);

          std::vector<cv::Point2f> corners = gate.getCorners();
          cv::circle(image, corners[0], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[1], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[2], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[3], 10, CV_RGB(0,255,0));
          cv::circle(image, center, 10, CV_RGB(0,0,255));
      }

      cv::imshow(OPENCV_WINDOW, image);
      cv::waitKey(3);

    }

public:

    //SampleImageConverter() : AbstractImageConverter() {}

    SampleImageConverter()
    {

      gatePublisher = nodeHandle.advertise<geometry_msgs::Pose2D>("/gate", 100);

      cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
    }

    ~SampleImageConverter()
    {
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

  ros::init(argc, argv, "gate_locator");
  SampleImageConverter ic;

  ros::spin();

  return 0;
}
