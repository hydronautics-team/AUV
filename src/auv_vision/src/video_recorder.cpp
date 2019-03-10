/*****************************************************************************
 * A fork of https://github.com/ros-perception/image_pipeline/blob/indigo/image_view/src/nodes/video_recorder.cpp
 * with support of slicing output video on 1 minute videos and multithreading.
 ****************************************************************************/

#include <ctime>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <opencv2/videoio.hpp>
#include <sys/stat.h>
#include <cstdlib>
#include <signal.h>

#include "../include/util/ConcurrentQueue.h"


cv::VideoWriter outputVideo;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
int fps;
std::string filename;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;

int slice_count = 1;
ros::Duration slice_duration = ros::Duration(60.0); // One minute duration of one slice
ros::Time first_frame_time = ros::Time(0);
bool first_frame = true;
bool recording = true;

ConcurrentQueue<sensor_msgs::ImageConstPtr> imageQueue;

std::string getCurrentDateTime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y_%H_%M_%S",timeinfo);
    std::string str(buffer);
    return  str;
}

void consume(ConcurrentQueue<sensor_msgs::ImageConstPtr>& queue)
{
    while (recording)
    {
        sensor_msgs::ImageConstPtr image_msg = queue.pop();

        if (!outputVideo.isOpened()) {

            std::string current_filename = filename + "_" + std::to_string(slice_count) + ".avi";
            slice_count++;

            cv::Size size(image_msg->width, image_msg->height);

            outputVideo.open(current_filename,
                             cv::VideoWriter::fourcc(
                                     codec.c_str()[0], codec.c_str()[1], codec.c_str()[2], codec.c_str()[3]),
                                     fps,
                                     size,
                                     true);

            if (!outputVideo.isOpened())
            {
                ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
                exit(-1);
            }

            g_last_wrote_time = ros::Time(0);
            g_count = 0;
            first_frame = true;

            ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

        }

        std::this_thread::sleep_for(std::chrono::milliseconds((int)((1.0 / fps) * 1000)));
        /**
        if ((image_msg->header.stamp - g_last_wrote_time) < ros::Duration(1 / fps))
        {
            continue;
        }
         */

        try {
            cv_bridge::CvtColorForDisplayOptions options;
            options.do_dynamic_scaling = use_dynamic_range;
            options.min_image_value = min_depth_range;
            options.max_image_value = max_depth_range;
            options.colormap = colormap;
            const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding, options)->image;
            if (!image.empty()) {
                outputVideo << image;
                g_count++;
                g_last_wrote_time = image_msg->header.stamp;
            } else {
                ROS_WARN("Frame skipped, no data!");
            }
        } catch(cv_bridge::Exception) {
            ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
            return;
        }
    }
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    imageQueue.push(image_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    std::string videoName;
    local_nh.param("filename", videoName, std::string("output"));

    filename = "/home/nvidia/records/" + videoName + "_" + getCurrentDateTime() + ".avi";

    bool stamped_filename;
    local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15);
    local_nh.param("codec", codec, std::string("MJPG"));
    local_nh.param("encoding", encoding, std::string("bgr8"));
    // cv_bridge::CvtColorForDisplayOptions
    local_nh.param("min_depth_range", min_depth_range, 0.0);
    local_nh.param("max_depth_range", max_depth_range, 0.0);
    local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    local_nh.param("colormap", colormap, -1);

    /* Not supported in this fork
    if (stamped_filename) {
        std::size_t found = filename.find_last_of("/\\");
        std::string path = filename.substr(0, found + 1);
        std::string basename = filename.substr(found + 1);
        std::stringstream ss;
        ss << ros::Time::now().toNSec() << basename;
        filename = path + ss.str();
        ROS_INFO("Video recording to %s", filename.c_str());
    }
    */

    // This fork supports only filename.avi output video name format
    std::size_t found = filename.find_last_of(".");
    filename = filename.substr(0, found) + "_";

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    image_transport::ImageTransport it(nh);
    std::string topic = nh.resolveName("image");
    image_transport::Subscriber sub_image = it.subscribe(topic, 1, callback);

    std::thread consumer(std::bind(consume, std::ref(imageQueue)));

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::spin();

    ROS_INFO("Releasing...");
    outputVideo.release();

    consumer.join();
}
