#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <msg/gate_msg.h>
#include <gate/GateDescriptor.h>
#include <gate/GateDetector.h>
#include "../include/AbstractImageConverter.h"

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

      gatePublisher.publish(gate.toMsg());

      if (gate.hasGate()) {
          std::vector<cv::Point2f> corners = gate.getCorners();
          cv::circle(image, corners[0], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[1], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[2], 10, CV_RGB(0,255,0));
          cv::circle(image, corners[3], 10, CV_RGB(0,255,0));
      }

      cv::imshow(OPENCV_WINDOW, image);
      cv::waitKey(3);

    }

public:

    SampleImageConverter()
    {
      gatePublisher = nodeHandle.advertise<auv_vision::gate_msg>("/gate", 100);

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
