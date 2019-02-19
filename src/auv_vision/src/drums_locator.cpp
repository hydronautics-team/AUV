#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <opencv2/opencv.hpp>
#include <util/ImgprocUtil.h>
#include "common/AbstractImageConverter.h"
#include "auv_common/OptionalPoint2D.h"
#include <mat/MatDetector.h>
#include <mat/MatDescriptor.h>
#include <drums/DrumDetector.h>
#include <drums/DrumDescriptor.h>
#include <auv_common/OptionalPoint2D.h>

// Gazebo:
static const std::string CAMERA_FRONT_TOPIC = "stereo/camera/left/image_raw";
static const std::string CAMERA_BOTTOM_TOPIC = "ROV_model_URDF/camera_bottom/image_raw";

// Real:
//static const std::string CAMERA_BOTTOM_TOPIC = "/cam_bottom/image_raw";
//static const std::string CAMERA_FRONT_TOPIC = "/cam_front_1/image_raw";

static const std::string DRUMS_MAT_PUBLISH_TOPIC_BOTTOM = "/drums/mat/cam_bottom";

static const std::string DRUMS_MAT_PUBLISH_TOPIC_FRONT = "/drums/mat/cam_front";

static const std::string DRUMS_DRUM_PUBLISH_TOPIC = "/drums/drum";

static const std::string DRUMS_LOCATOR_NODE_NAME = "drums_locator";

static const std::string OPENCV_WINDOW_FRONT_CAM_MAT = "Front Camera Mat Detection";
static const std::string OPENCV_WINDOW_BOTTOM_CAM_MAT = "Bottom Camera";
static const std::string OPENCV_WINDOW_DRUM = "Bottom Camera Drum Detection";

class MatPublisherFrontCam : public AbstractImageConverter
{

private:

    ros::Publisher matFrontCamPublisher;

    MatDetector detector;
    MatDetectorFrontCamera detector_frontCamera;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr) {
        cv::Mat image = cv_ptr->image;

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat image_copy = image.clone();

        cv::Mat image_copy_descriptor_front = image.clone();

        auv_common::OptionalPoint2D msg_front_cam;

        if (!image.empty()) detector.detectContours(image, image_copy, contours, true);

        /// Initialising MatDescriptorFrontCamera's object front_image via implementing method detect (class MatDetectorFrontCamera)
        /// (returns MatDescriptorFrontCamera::create(contours) or returns MatDescriptorFrontCamera::noMat) with parameters (bool mat, const std::vector<std::vector<cv::Point>>& contour)
        MatDescriptorFrontCamera front_image = detector_frontCamera.detect(image_copy, image_copy, contours);

        //cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black

        if (front_image.hasMat()) {
            std::vector<std::vector<cv::Point>> contour = front_image.getContour();

            cv::RNG rng;
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                          rng.uniform(0, 255)); /// Random colors
            //cv::drawContours(drawing, contour, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

            //cv::namedWindow("Biggest Contour IN MAIN");
            //if (!drawing.empty()) cv::imshow("Biggest Contour IN MAIN", drawing);

            cv::Point2f center = front_image.getCenter();
            cv::circle(image_copy_descriptor_front, center, 3, cv::Scalar(0, 0, 255));
            center = convertToCentralCoordinates(center, image.cols, image.rows);
            //std::cout<<center<<" THIS IS MAIN"<<std::endl;

            msg_front_cam.hasPoint = true;
            msg_front_cam.x = center.x;
            msg_front_cam.y = center.y;
            matFrontCamPublisher.publish(msg_front_cam);

            color = cv::Scalar(255, 255, 0);
            cv::rectangle(image_copy_descriptor_front, front_image.getBoundingRect().tl(),
                          front_image.getBoundingRect().br(), color, 2, 8, 0);


        }

        if (!image_copy_descriptor_front.empty()) cv::imshow(OPENCV_WINDOW_FRONT_CAM_MAT, image_copy_descriptor_front);
        cv::waitKey(3);
    }

public:

    /// Constructor
    MatPublisherFrontCam(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_FRONT ("/drums/mat/cam_front").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_FRONT
         * that we are going to publish data on that topic.
         */
        matFrontCamPublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(DRUMS_MAT_PUBLISH_TOPIC_FRONT, 100);
        cv::namedWindow(OPENCV_WINDOW_FRONT_CAM_MAT, CV_WINDOW_AUTOSIZE);
    }

    /// Destructor
    ~MatPublisherFrontCam()
    {
    }

};

cv::Mat image_copy_descriptor_bottom;

class MatPublisherBottomCam : public AbstractImageConverter
{

private:

    ros::Publisher matBottomCamPublisher;

    MatDetector detector;
    MatDetectorBottomCamera detector_bottomCamera;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr) {
        cv::Mat image = cv_ptr->image;

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat image_copy = image.clone();
        image_copy_descriptor_bottom = image.clone();

        auv_common::OptionalPoint2D msg_bottom_cam;

        if (!image.empty()) detector.detectContours(image, image_copy, contours, true);

        MatDescriptorBottomCamera bottom_image = detector_bottomCamera.detect(image_copy, image_copy);

        //cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black

        if (bottom_image.hasHorizontalLines()) {

            /// Draw the lines
            for (int i = 0; i < bottom_image.getHorizontalLines().size(); i++) {
                cv::Vec4f l = bottom_image.getHorizontalLines()[i];
                //cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                //cv::line(drawing, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 2, CV_AA);
            }

            //cv::namedWindow("Horizontal Lines");
            //if (!drawing.empty()) cv::imshow("Horizontal Lines", drawing);


            /// Expand the lines
            for (int i = 0; i < bottom_image.getHorizontalLines().size(); i++) {
                /*
                cv::Vec4f v = bottom_image.getHorizontalLines()[i];
                bottom_image.getHorizontalLines()[i][0] = 0;
                bottom_image.getHorizontalLines()[i][1] = ((float) v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
                bottom_image.getHorizontalLines()[i][2] = image.cols;
                bottom_image.getHorizontalLines()[i][3] =
                        ((float) v[1] - v[3]) / (v[0] - v[2]) * (image.cols - v[2]) + v[3];
                */

                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                cv::line(image_copy_descriptor_bottom,
                         cv::Point(bottom_image.getHorizontalLines()[i][0], bottom_image.getHorizontalLines()[i][1]),
                         cv::Point(bottom_image.getHorizontalLines()[i][2], bottom_image.getHorizontalLines()[i][3]),
                         color, 2, CV_AA);

                std::cout<<bottom_image.getIntersectionWithHorizontal(image, bottom_image.getHorizontalLines()[i])<<" Intersection Y"<<std::endl;
            }
        }
        //drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black

        if (bottom_image.hasVerticalLines()) {

            /// Draw the lines
            for (int i = 0; i < bottom_image.getVerticalLines().size(); i++) {
                cv::Vec4f l = bottom_image.getVerticalLines()[i];
                //cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                //cv::line(drawing, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 2, CV_AA);
            }

            //cv::namedWindow("Vertical Lines");
            //if (!drawing.empty()) cv::imshow("Vertical Lines", drawing);


            /// Expand the lines
            for (int i = 0; i < bottom_image.getVerticalLines().size(); i++) {
                /*
                cv::Vec4f v = bottom_image.getVerticalLines()[i];
                bottom_image.getVerticalLines()[i][0] = 0;
                bottom_image.getVerticalLines()[i][1] = ((float) v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
                bottom_image.getVerticalLines()[i][2] = image.cols;
                bottom_image.getVerticalLines()[i][3] =
                        ((float) v[1] - v[3]) / (v[0] - v[2]) * (image.cols - v[2]) + v[3];
                */

                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                cv::line(image_copy_descriptor_bottom,
                         cv::Point(bottom_image.getVerticalLines()[i][0], bottom_image.getVerticalLines()[i][1]),
                         cv::Point(bottom_image.getVerticalLines()[i][2], bottom_image.getVerticalLines()[i][3]),
                         color, 2, CV_AA);

                std::cout<<bottom_image.getIntersectionWithVertical(image, bottom_image.getVerticalLines()[i])<<" Intersection X"<<std::endl;
            }

            //cv::namedWindow("Lines Expanded");
            //if (!image_copy_descriptor_bottom.empty()) cv::imshow("Lines Expanded", image_copy_descriptor_bottom);
        }
        //if (!image_copy_descriptor_bottom.empty()) cv::imshow(OPENCV_WINDOW_BOTTOM_CAM_MAT, image_copy_descriptor_bottom);
        //cv::waitKey(3);
    }

public:

    /// Constructor
    MatPublisherBottomCam(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_BOTTOM ("/drums/mat/cam_bottom").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_BOTTOM
         * that we are going to publish data on that topic.
         */
        matBottomCamPublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(DRUMS_MAT_PUBLISH_TOPIC_BOTTOM, 100);
        cv::namedWindow(OPENCV_WINDOW_BOTTOM_CAM_MAT, CV_WINDOW_AUTOSIZE);
    }

    /// Destructor
    ~MatPublisherBottomCam()
    {
    }

};

class DrumPublisher : public AbstractImageConverter
{

private:

    ros::Publisher drumPublisher;

    DrumDetector Drum_detector;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        cv::Mat image = cv_ptr->image;

        DrumDescriptor image_with_Drums = Drum_detector.detect(image, false);

        cv::Mat image_copy_descriptor_Drums = image.clone();

        if (image_with_Drums.hasDrumCenter()) {
            if ((image_with_Drums.hasBlueDrumCenter()) && (image_with_Drums.hasRedDrumCenter())) {
                std::vector<cv::Vec3f> RedDrums = image_with_Drums.getAllRedDrums();
                std::vector<cv::Vec3f> BlueDrums = image_with_Drums.getAllBlueDrums();

                for (int i = 0; i < RedDrums.size(); i++) {
                    cv::Point2f RedDrumCenter = image_with_Drums.getRedDrumCenter(i);
                    if ((RedDrumCenter.x != 0) && (RedDrumCenter.y != 0)) {
                        /// Draw
                        cv::circle (image_copy_descriptor_Drums, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_Drums, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                        cv::circle (image_copy_descriptor_bottom, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_bottom, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                        RedDrumCenter = convertToCentralCoordinates(RedDrumCenter, image.cols, image.rows);
                        std::cout<<RedDrumCenter<<" THIS IS MAIN RED"<<std::endl;
                    }
                }

                for (int i = 0; i < BlueDrums.size(); i++) {
                    cv::Point2f BlueDrumCenter = image_with_Drums.getBlueDrumCenter(i);

                    if ((BlueDrumCenter.x != 0) && (BlueDrumCenter.y != 0)) {
                        /// Draw
                        cv::circle (image_copy_descriptor_Drums, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_Drums, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                        cv::circle (image_copy_descriptor_bottom, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_bottom, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                        BlueDrumCenter = convertToCentralCoordinates(BlueDrumCenter, image.cols, image.rows);

                        std::cout<<BlueDrumCenter<<" THIS IS MAIN BlUE"<<std::endl;
                    }
                }
            }
            else {
                if (image_with_Drums.hasBlueDrumCenter()) {
                    std::vector<cv::Vec3f> BlueDrums = image_with_Drums.getAllBlueDrums();

                    for (int i = 0; i < BlueDrums.size(); i++) {
                        cv::Point2f BlueDrumCenter = image_with_Drums.getBlueDrumCenter(i);

                        if ((BlueDrumCenter.x != 0) && (BlueDrumCenter.y != 0)) {
                            /// Draw
                            cv::circle(image_copy_descriptor_Drums, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle(image_copy_descriptor_Drums, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                            cv::circle(image_copy_descriptor_bottom, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle(image_copy_descriptor_bottom, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                            BlueDrumCenter = convertToCentralCoordinates(BlueDrumCenter, image.cols, image.rows);

                            std::cout << BlueDrumCenter << " THIS IS MAIN BlUE 1" << std::endl;
                        }
                    }
                }
                if (image_with_Drums.hasRedDrumCenter()) {
                    std::vector<cv::Vec3f> RedDrums = image_with_Drums.getAllRedDrums();

                    for (int i = 0; i < RedDrums.size(); i++) {
                        cv::Point2f RedDrumCenter = image_with_Drums.getRedDrumCenter(i);

                        if ((RedDrumCenter.x != 0) && (RedDrumCenter.y != 0)) {
                            /// Draw
                            cv::circle (image_copy_descriptor_Drums, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle (image_copy_descriptor_Drums, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                            cv::circle (image_copy_descriptor_bottom, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle (image_copy_descriptor_bottom, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                            RedDrumCenter = convertToCentralCoordinates(RedDrumCenter, image.cols, image.rows);
                            std::cout<<RedDrumCenter<<" THIS IS MAIN RED 1"<<std::endl;

                            auv_common::OptionalPoint2D msg_bottom_cam;

                            msg_bottom_cam.hasPoint = true;
                            msg_bottom_cam.x = RedDrumCenter.x;
                            msg_bottom_cam.y = RedDrumCenter.y;
                            drumPublisher.publish(msg_bottom_cam);

                        }
                    }
                }
            }

            //cv::namedWindow("Drums");
            //if (!image_copy_descriptor_Drums.empty()) cv::imshow("Drums", image_copy_descriptor_Drums);

        }

        else {
            std::cerr<<"NO Drums"<<std::endl;

            auv_common::OptionalPoint2D msg_bottom_cam;

            msg_bottom_cam.hasPoint = false;
            msg_bottom_cam.x = 0;
            msg_bottom_cam.y = 0;
            drumPublisher.publish(msg_bottom_cam);
            ///STUFF
        }
        if (!image_copy_descriptor_Drums.empty()) cv::imshow(OPENCV_WINDOW_DRUM, image_copy_descriptor_Drums);
        if (!image_copy_descriptor_bottom.empty()) cv::imshow(OPENCV_WINDOW_BOTTOM_CAM_MAT, image_copy_descriptor_bottom);
        cv::waitKey(3);
    }

public:

    DrumPublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        drumPublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(DRUMS_DRUM_PUBLISH_TOPIC, 100);
    }

    ~DrumPublisher()
    {
    }

};


int main(int argc, char **argv)
{

    #if CV_MAJOR_VERSION == 2
        ROS_INFO("%s", "OpenCV version = 2");
    #elif CV_MAJOR_VERSION == 3
        ROS_INFO("%s", "OpenCV version = 3");
    #endif

    ros::init(argc, argv, DRUMS_LOCATOR_NODE_NAME);

    MatPublisherFrontCam matPublisherFrontCam(CAMERA_FRONT_TOPIC);
    MatPublisherBottomCam matPublisherBottomCam(CAMERA_BOTTOM_TOPIC);

    DrumPublisher drumPublisher(CAMERA_BOTTOM_TOPIC);

    ros::spin();

    return 0;
}
