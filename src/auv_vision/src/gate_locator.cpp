#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "../include/AbstractImageConverter.h"

static const std::string OPENCV_WINDOW = "Image window";

class SampleImageConverter : public AbstractImageConverter
{

private:

    image_transport::Publisher imagePublisher;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
      cv::Mat image = cv_ptr->image;

      // Draw an example circle on the video stream
      if (image.rows > 60 && image.cols > 60)
        cv::circle(image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, image);
      cv::waitKey(3);

      // Output modified video stream
      imagePublisher.publish(cv_ptr->toImageMsg());
    }

public:

    SampleImageConverter()
    {
      imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);
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
