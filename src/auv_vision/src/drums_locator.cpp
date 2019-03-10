#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <opencv2/opencv.hpp>
#include <util/ImgprocUtil.h>
#include "common/AbstractImageConverter.h"
#include "auv_common/OptionalPoint2D.h"
#include "auv_common/DistancesToMatEdges.h"
#include "auv_common/DrumsCoordinates.h"
#include "mat/MatDetector.h"
#include "mat/MatDescriptor.h"
#include "drums/DrumDetector.h"
#include "drums/DrumDescriptor.h"
#include <dynamic_reconfigure/server.h>
#include <auv_vision/DrumsLocatorConfig.h>


static const std::string CAMERA_BOTTOM_TOPIC = "/cam_bottom/image_raw";
static const std::string CAMERA_FRONT_TOPIC = "/cam_front_1/image_raw";

static const std::string DRUMS_MAT_PUBLISH_TOPIC_BOTTOM = "/drums/mat/cam_bottom";

static const std::string DRUMS_MAT_PUBLISH_TOPIC_FRONT = "/drums/mat/cam_front";

static const std::string ENABLE_WINDOWS_PARAM = "debugVision";

static const std::string DRUMS_DRUM_PUBLISH_TOPIC = "/drums/drum";
static const std::string DRUMS_DRUM_PUBLISH_TOPIC_FRONT_CAM = "/drums/cam_front";

static const std::string DRUMS_LOCATOR_NODE_NAME = "drums_locator";

static const std::string OPENCV_WINDOW_FRONT_CAM_MAT = "Front Camera Mat Detection";
static const std::string OPENCV_WINDOW_FRONT_CAM_DRUM = "Front Camera Drum Detection";
static const std::string OPENCV_WINDOW_BOTTOM_CAM_MAT = "Bottom Camera";
static const std::string OPENCV_WINDOW_DRUM = "Bottom Camera Drum Detection";

MatDetector detector;
MatDetectorFrontCamera detector_frontCamera;
MatDetectorBottomCamera detector_bottomCamera;
FrontCameraDrumDetector detector_frontCameraDrum;
DrumDetector drum_detector;

void reconfigureDrums(auv_vision::DrumsLocatorConfig& config, uint32_t level) {

                    /// Front Camera mat
                    detector.setLowerGreenH(config.lowerGreenH);
                    detector.setHigherGreenH(config.higherGreenH);

                    detector.setLowerGreenS(config.lowerGreenS);
                    detector.setHigherGreenS(config.higherGreenS);

                    detector.setLowerGreenV(config.lowerGreenV);
                    detector.setHigherGreenV(config.higherGreenV);


                    /// Bottom camera mat
                    detector_bottomCamera.setLengthThreshold(config.lengthThreshold);
                    detector_bottomCamera.setDistanceThreshold(config.distanceThreshold);
                    detector_bottomCamera.setMinAngleCriteria(config.minAngleCriteria);
                    detector_bottomCamera.setMaxAngleCriteria(config.maxAngleCriteria);


                    /// Bottom camera drums
                    drum_detector.setLowerBlueH(config.lowerBlueH);
                    drum_detector.setHigherBlueH(config.higherBlueH);
                    drum_detector.setLowerBlueS(config.lowerBlueS);
                    drum_detector.setHigherBlueS(config.higherBlueS);
                    drum_detector.setLowerBlueV(config.lowerBlueV);
                    drum_detector.setHigherBlueV(config.higherBlueV);


                    drum_detector.setLowerRED_1H(config.lowerRed1H);
                    drum_detector.setHigherRED_1H(config.higherRed1H);
                    drum_detector.setLowerRED_2H(config.lowerRed2H);
                    drum_detector.setHigherRED_2H(config.higherRed2H);

                    drum_detector.setLowerRED_1S(config.lowerRed1S);
                    drum_detector.setHigherRED_1S(config.higherRed1S);
                    drum_detector.setLowerRED_2S(config.lowerRed2S);
                    drum_detector.setHigherRED_2S(config.higherRed2S);

                    drum_detector.setLowerRED_1V(config.lowerRed1V);
                    drum_detector.setHigherRED_1V(config.higherRed1V);
                    drum_detector.setLowerRED_2V(config.lowerRed2V);
                    drum_detector.setHigherRED_2V(config.higherRed2V);


                    drum_detector.setMinDist(config.minDist);
                    drum_detector.setParam1(config.param1);
                    drum_detector.setParam2(config.param2);
}

class MatPublisherFrontCam : public AbstractImageConverter
{

private:

    ros::Publisher matFrontCamPublisher;

    image_transport::Publisher imagePublisher, maskedImagePublisher, linesImagePublisher, contoursImagePublisher;

    bool windowsEnabled;

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
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)); /// Random colors
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
        // TODO enable this when concurrence state is fully OK
        /*
        else {
            msg_front_cam.hasPoint = false;
            matFrontCamPublisher.publish(msg_front_cam);
        }
        */

        if (windowsEnabled) {
            if (!image_copy_descriptor_front.empty()) cv::imshow(OPENCV_WINDOW_FRONT_CAM_MAT, image_copy_descriptor_front);
            cv::waitKey(3);
        }


        if (!image_copy_descriptor_front.empty()) {
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy_descriptor_front).toImageMsg();
            imagePublisher.publish(imageMsg);
        }

        if (!detector.getLinesImage().empty()) {
            sensor_msgs::ImagePtr imageMsgLines = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detector.getLinesImage()).toImageMsg();
            linesImagePublisher.publish(imageMsgLines);
        }

        if (!detector.getimageAfterContourDetection().empty()) {
            sensor_msgs::ImagePtr imageMsgContours = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detector.getimageAfterContourDetection()).toImageMsg();
            contoursImagePublisher.publish(imageMsgContours);
        }

        if (!image_copy.empty()) {
            sensor_msgs::ImagePtr imageMsgMasked = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_copy).toImageMsg();
            maskedImagePublisher.publish(imageMsgMasked);
        }
    }

public:

    /// Constructor
    MatPublisherFrontCam(const std::string& inputImageTopic, bool enableWindows) : AbstractImageConverter(inputImageTopic),
                                                                                   windowsEnabled(enableWindows) {

        image_transport::ImageTransport it(nodeHandle);
        linesImagePublisher = it.advertise("/frontCamMat/image_lines", 100);
        maskedImagePublisher = it.advertise("/frontCamMat/image_masked", 100);
        imagePublisher = it.advertise("/frontCamMat/image", 100);
        contoursImagePublisher = it.advertise("/frontCamMat/image_contours", 100);

        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_FRONT ("/drums/mat/cam_front").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_FRONT
         * that we are going to publish data on that topic.
         */

        matFrontCamPublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(DRUMS_MAT_PUBLISH_TOPIC_FRONT, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW_FRONT_CAM_MAT, CV_WINDOW_AUTOSIZE);

    }

    /// Destructor
    ~MatPublisherFrontCam()
    {
        if (windowsEnabled)
            cv::destroyWindow(OPENCV_WINDOW_FRONT_CAM_MAT);
    }

};

class DrumPublisherFrontCam : public AbstractImageConverter
{

private:

    ros::Publisher drumFrontCamPublisher;

    image_transport::Publisher imagePublisher, maskedImagePublisher, contoursImagePublisher;

    bool windowsEnabled;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr) {
        cv::Mat image = cv_ptr->image;

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat image_copy = image.clone();
        cv::Mat image_copy_descriptor_front = image.clone();

        auv_common::OptionalPoint2D msg_front_cam_drum;

        /// Initialising MatDescriptorFrontCamera's object front_image via implementing method detect (class MatDetectorFrontCamera)
        /// (returns MatDescriptorFrontCamera::create(contours) or returns MatDescriptorFrontCamera::noMat) with parameters (bool mat, const std::vector<std::vector<cv::Point>>& contour)
        FrontCameraDrumDescriptor front_image_drum = detector_frontCameraDrum.detect(image_copy, image_copy);

        //cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black

        if (front_image_drum.hasDrum()) {
            std::vector<std::vector<cv::Point>> contour = front_image_drum.getContour();

            cv::RNG rng;
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)); /// Random colors
            //cv::drawContours(drawing, contour, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

            //cv::namedWindow("Biggest Contour IN MAIN");
            //if (!drawing.empty()) cv::imshow("Biggest Contour IN MAIN", drawing);

            cv::Point2f center = front_image_drum.getCenter();
            cv::circle(image_copy_descriptor_front, center, 3, cv::Scalar(0, 0, 255));
            center = convertToCentralCoordinates(center, image.cols, image.rows);
            //std::cout<<center<<" THIS IS MAIN"<<std::endl;

            msg_front_cam_drum.hasPoint = true;
            msg_front_cam_drum.x = center.x;
            msg_front_cam_drum.y = center.y;
            drumFrontCamPublisher.publish(msg_front_cam_drum);

            color = cv::Scalar(255, 255, 0);
            cv::rectangle(image_copy_descriptor_front, front_image_drum.getBoundingRect().tl(),
                          front_image_drum.getBoundingRect().br(), color, 2, 8, 0);


        }
        // TODO enable this when concurrence state is fully OK
        /*
        else {
            msg_front_cam.hasPoint = false;
            matFrontCamPublisher.publish(msg_front_cam);
        }
        */

        if (windowsEnabled) {
            if (!image_copy_descriptor_front.empty()) cv::imshow(OPENCV_WINDOW_FRONT_CAM_DRUM, image_copy_descriptor_front);
            cv::waitKey(3);
        }


        if (!image_copy_descriptor_front.empty()) {
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy_descriptor_front).toImageMsg();
            imagePublisher.publish(imageMsg);
        }

        if (!detector.getimageAfterContourDetection().empty()) {
            sensor_msgs::ImagePtr imageMsgContours = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detector_frontCameraDrum.getimageAfterContourDetectionDrum()).toImageMsg();
            contoursImagePublisher.publish(imageMsgContours);
        }

        if (!detector_frontCameraDrum.getimageAfterMaskDrum().empty()) {
            sensor_msgs::ImagePtr imageMsgMasked = cv_bridge::CvImage(std_msgs::Header(), "mono8", detector_frontCameraDrum.getimageAfterMaskDrum()).toImageMsg();
            maskedImagePublisher.publish(imageMsgMasked);
        }
    }

public:

    /// Constructor
    DrumPublisherFrontCam(const std::string& inputImageTopic, bool enableWindows) : AbstractImageConverter(inputImageTopic),
                                                                                   windowsEnabled(enableWindows) {

        image_transport::ImageTransport it(nodeHandle);
        maskedImagePublisher = it.advertise("/drums/drum_front/image_masked", 100);
        imagePublisher = it.advertise("/drums/drum_front/image", 100);
        contoursImagePublisher = it.advertise("/drums/drum_front/image_contours", 100);

        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_FRONT ("/drums/mat/cam_front").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_FRONT
         * that we are going to publish data on that topic.
         */

        drumFrontCamPublisher = nodeHandle.advertise<auv_common::OptionalPoint2D>(DRUMS_DRUM_PUBLISH_TOPIC_FRONT_CAM, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW_FRONT_CAM_DRUM, CV_WINDOW_AUTOSIZE);

    }

    /// Destructor
    ~DrumPublisherFrontCam()
    {
        if (windowsEnabled)
            cv::destroyWindow(OPENCV_WINDOW_FRONT_CAM_DRUM);
    }

};

cv::Mat image_copy_descriptor_bottom;

class MatPublisherBottomCam : public AbstractImageConverter
{

private:

    ros::Publisher matBottomCamPublisher;

    image_transport::Publisher imagePublisher, maskedImagePublisher, allLinesImagePublisher;

    bool windowsEnabled;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr) {
        cv::Mat image = cv_ptr->image;

        std::vector<std::vector<cv::Point>> contours;
        cv::Mat image_copy = image.clone();
        image_copy_descriptor_bottom = image.clone();

        auv_common::DistancesToMatEdges msg_bottom_cam;

        if (!image.empty()) {
            detector.detectContours(image, image_copy, contours, true);

            if (!image_copy.empty()) {
                sensor_msgs::ImagePtr imageMsgMasked = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_copy).toImageMsg();
                maskedImagePublisher.publish(imageMsgMasked);
            }
        }

        MatDescriptorBottomCamera bottom_image = detector_bottomCamera.detect(image_copy, image_copy);

        msg_bottom_cam.hasHorizontalLine = false;
        msg_bottom_cam.hasVerticalLine = false;
        msg_bottom_cam.distanceToHorizontalLine = -1;
        msg_bottom_cam.distanceToVerticalLine = -1;

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

                std::cerr<<bottom_image.getIntersectionWithHorizontal(image, bottom_image.getHorizontalLines()[i])<<" Intersection Y"<<std::endl;

                msg_bottom_cam.hasHorizontalLine = true;
                msg_bottom_cam.distanceToHorizontalLine = bottom_image.getIntersectionWithHorizontal(image, bottom_image.getHorizontalLines()[i]);
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

                std::cerr<<bottom_image.getIntersectionWithVertical(image, bottom_image.getVerticalLines()[i])<<" Intersection X"<<std::endl;

                msg_bottom_cam.hasVerticalLine = true;
                msg_bottom_cam.distanceToVerticalLine = bottom_image.getIntersectionWithVertical(image, bottom_image.getVerticalLines()[i]);
            }

            //cv::namedWindow("Lines Expanded");
            //if (!image_copy_descriptor_bottom.empty()) cv::imshow("Lines Expanded", image_copy_descriptor_bottom);
        }
        //if (!image_copy_descriptor_bottom.empty()) cv::imshow(OPENCV_WINDOW_BOTTOM_CAM_MAT, image_copy_descriptor_bottom);
        //cv::waitKey(3);

        matBottomCamPublisher.publish(msg_bottom_cam);

        if (!image_copy_descriptor_bottom.empty()) {
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy_descriptor_bottom).toImageMsg();
            imagePublisher.publish(imageMsg);
        }

        if (!detector_bottomCamera.getimageWithAllLines().empty()) {
            sensor_msgs::ImagePtr imageMsgAllLines = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detector_bottomCamera.getimageWithAllLines()).toImageMsg();
            allLinesImagePublisher.publish(imageMsgAllLines);
        }
    }

public:

    /// Constructor
    MatPublisherBottomCam(const std::string& inputImageTopic, bool enableWindows) : AbstractImageConverter(inputImageTopic), windowsEnabled(enableWindows) {

        image_transport::ImageTransport it(nodeHandle);
        imagePublisher = it.advertise("/bottomCamMat/image", 100);
        maskedImagePublisher = it.advertise("/bottomCamMat/image_masked", 100);
        allLinesImagePublisher = it.advertise("/bottomCamMat/image_all_lines", 100);

        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_BOTTOM ("/drums/mat/cam_bottom").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_BOTTOM
         * that we are going to publish data on that topic.
         */
        matBottomCamPublisher = nodeHandle.advertise<auv_common::DistancesToMatEdges>(DRUMS_MAT_PUBLISH_TOPIC_BOTTOM, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW_BOTTOM_CAM_MAT, CV_WINDOW_AUTOSIZE);

    }

    /// Destructor
    ~MatPublisherBottomCam()
    {
        if (windowsEnabled)
            cv::destroyWindow(OPENCV_WINDOW_BOTTOM_CAM_MAT);
    }

};

class DrumPublisher : public AbstractImageConverter
{

private:

    ros::Publisher drumPublisher;

    image_transport::Publisher imagePublisher, imagePublisherAfterMorphology, imagePublisherAfterMask, imagePublisherAfterMaskRED, imagePublisherAfterMaskBLUE, imagePublisherAfterColorEnhancement, imagePublisherImage_red, imagePublisherImage_blue;

    bool windowsEnabled;

protected:

    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        cv::Mat image = cv_ptr->image;

        DrumDescriptor image_with_Drums = drum_detector.detect(image, false);

        cv::Mat image_copy_descriptor_Drums = image.clone();

        auv_common::DrumsCoordinates msg_bottom_cam_drums;

        if (image_with_Drums.hasDrumCenter()) {
            if ((image_with_Drums.hasBlueDrumCenter()) && (image_with_Drums.hasRedDrumCenter())) {
                std::vector<cv::Vec3f> RedDrums = image_with_Drums.getAllRedDrums();
                std::vector<cv::Vec3f> BlueDrums = image_with_Drums.getAllBlueDrums();

                bool stupidFlag = true;
                for (int i = 0; i < RedDrums.size(); i++) {
                    cv::Point2f RedDrumCenter = image_with_Drums.getRedDrumCenter(i);
                    if ((RedDrumCenter.x != 0) && (RedDrumCenter.y != 0)) {
                        /// Draw
                        cv::circle (image_copy_descriptor_Drums, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_Drums, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                        cv::circle (image_copy_descriptor_bottom, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_bottom, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                        RedDrumCenter = convertToCentralCoordinates(RedDrumCenter, image.cols, image.rows);
                        std::cerr<<RedDrumCenter<<"RED"<<std::endl;
                        msg_bottom_cam_drums.hasRedDrum = true;
                        if (stupidFlag) {
                            stupidFlag = false;
                            msg_bottom_cam_drums.RedDrum1x = RedDrumCenter.x;
                            msg_bottom_cam_drums.RedDrum1y = RedDrumCenter.y;
                        }
                        else {
                            msg_bottom_cam_drums.RedDrum2x = RedDrumCenter.x;
                            msg_bottom_cam_drums.RedDrum2y = RedDrumCenter.y;
                        }
                    }
                }

                stupidFlag = true;
                for (int i = 0; i < BlueDrums.size(); i++) {
                    cv::Point2f BlueDrumCenter = image_with_Drums.getBlueDrumCenter(i);

                    if ((BlueDrumCenter.x != 0) && (BlueDrumCenter.y != 0)) {
                        /// Draw
                        cv::circle (image_copy_descriptor_Drums, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_Drums, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                        cv::circle (image_copy_descriptor_bottom, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                        cv::circle (image_copy_descriptor_bottom, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                        BlueDrumCenter = convertToCentralCoordinates(BlueDrumCenter, image.cols, image.rows);

                        std::cerr<<BlueDrumCenter<<"BlUE"<<std::endl;
                        msg_bottom_cam_drums.hasBlueDrum = true;
                        if (stupidFlag) {
                            stupidFlag = false;
                            msg_bottom_cam_drums.BlueDrum1x = BlueDrumCenter.x;
                            msg_bottom_cam_drums.BlueDrum1y = BlueDrumCenter.y;
                        }
                        else {
                            msg_bottom_cam_drums.BlueDrum2x = BlueDrumCenter.x;
                            msg_bottom_cam_drums.BlueDrum2y = BlueDrumCenter.y;
                        }
                    }
                }

            }
            else {
                if (image_with_Drums.hasBlueDrumCenter()) {
                    std::vector<cv::Vec3f> BlueDrums = image_with_Drums.getAllBlueDrums();

                    bool stupidFlag = true;
                    for (int i = 0; i < BlueDrums.size(); i++) {
                        cv::Point2f BlueDrumCenter = image_with_Drums.getBlueDrumCenter(i);

                        if ((BlueDrumCenter.x != 0) && (BlueDrumCenter.y != 0)) {
                            /// Draw
                            cv::circle(image_copy_descriptor_Drums, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle(image_copy_descriptor_Drums, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                            cv::circle(image_copy_descriptor_bottom, BlueDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle(image_copy_descriptor_bottom, BlueDrumCenter, BlueDrums[i][2], cv::Scalar(0, 0, 255));

                            BlueDrumCenter = convertToCentralCoordinates(BlueDrumCenter, image.cols, image.rows);

                            std::cerr << BlueDrumCenter << "BlUE" << std::endl;

                            msg_bottom_cam_drums.hasBlueDrum = true;
                            if (stupidFlag) {
                                stupidFlag = false;
                                msg_bottom_cam_drums.BlueDrum1x = BlueDrumCenter.x;
                                msg_bottom_cam_drums.BlueDrum1y = BlueDrumCenter.y;
                            }
                            else {
                                msg_bottom_cam_drums.BlueDrum2x = BlueDrumCenter.x;
                                msg_bottom_cam_drums.BlueDrum2y = BlueDrumCenter.y;
                            }
                        }
                    }
                }
                if (image_with_Drums.hasRedDrumCenter()) {
                    std::vector<cv::Vec3f> RedDrums = image_with_Drums.getAllRedDrums();

                    bool stupidFlag = true;
                    for (int i = 0; i < RedDrums.size(); i++) {
                        cv::Point2f RedDrumCenter = image_with_Drums.getRedDrumCenter(i);

                        if ((RedDrumCenter.x != 0) && (RedDrumCenter.y != 0)) {
                            /// Draw
                            cv::circle (image_copy_descriptor_Drums, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle (image_copy_descriptor_Drums, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                            cv::circle (image_copy_descriptor_bottom, RedDrumCenter, 3, cv::Scalar(0, 255, 0));
                            cv::circle (image_copy_descriptor_bottom, RedDrumCenter, RedDrums[i][2], cv::Scalar(255, 0, 0));

                            RedDrumCenter = convertToCentralCoordinates(RedDrumCenter, image.cols, image.rows);
                            std::cerr<<RedDrumCenter<<"RED"<<std::endl;

                            auv_common::OptionalPoint2D msg_bottom_cam;

                            msg_bottom_cam_drums.hasRedDrum = true;
                            if (stupidFlag) {
                                stupidFlag = false;
                                msg_bottom_cam_drums.RedDrum1x = RedDrumCenter.x;
                                msg_bottom_cam_drums.RedDrum1y = RedDrumCenter.y;
                            }
                            else {
                                msg_bottom_cam_drums.RedDrum2x = RedDrumCenter.x;
                                msg_bottom_cam_drums.RedDrum2y = RedDrumCenter.y;
                            }
                        }
                    }
                }
            }

            //cv::namedWindow("Drums");
            //if (!image_copy_descriptor_Drums.empty()) cv::imshow("Drums", image_copy_descriptor_Drums);

        }

        else {
            //std::cerr<<"NO Drums"<<std::endl;
            msg_bottom_cam_drums.hasRedDrum = false;
            msg_bottom_cam_drums.hasBlueDrum = false;

            msg_bottom_cam_drums.RedDrum1x = -1;
            msg_bottom_cam_drums.RedDrum1y = -1;
            msg_bottom_cam_drums.RedDrum2x = -1;
            msg_bottom_cam_drums.RedDrum2y = -1;

            msg_bottom_cam_drums.BlueDrum1x = -1;
            msg_bottom_cam_drums.BlueDrum1y = -1;
            msg_bottom_cam_drums.BlueDrum2x = -1;
            msg_bottom_cam_drums.BlueDrum2y = -1;

        }
        drumPublisher.publish(msg_bottom_cam_drums);

        if (windowsEnabled) {
            if (!image_copy_descriptor_Drums.empty()) cv::imshow(OPENCV_WINDOW_DRUM, image_copy_descriptor_Drums);
            if (!image_copy_descriptor_bottom.empty()) cv::imshow(OPENCV_WINDOW_BOTTOM_CAM_MAT, image_copy_descriptor_bottom);
            cv::waitKey(3);
        }

        if (!image_copy_descriptor_bottom.empty()) {
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy_descriptor_bottom).toImageMsg();
            imagePublisher.publish(imageMsg);
        }

        if (!drum_detector.getreconfImageAfterMorphology().empty()) {
            sensor_msgs::ImagePtr imageMsgAfterMorphology = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drum_detector.getreconfImageAfterMorphology()).toImageMsg();
            imagePublisherAfterMorphology.publish(imageMsgAfterMorphology);
        }

        if (!drum_detector.getreconfImageAfterMask().empty()) {
            sensor_msgs::ImagePtr imageMsgAfterMask = cv_bridge::CvImage(std_msgs::Header(), "mono8", drum_detector.getreconfImageAfterMask()).toImageMsg();
            imagePublisherAfterMask.publish(imageMsgAfterMask);
        }

        if (!drum_detector.getreconfImageAfterMaskRED().empty()) {
            sensor_msgs::ImagePtr imageMsgAfterMaskRED = cv_bridge::CvImage(std_msgs::Header(), "mono8", drum_detector.getreconfImageAfterMaskRED()).toImageMsg();
            imagePublisherAfterMaskRED.publish(imageMsgAfterMaskRED);
        }

        if (!drum_detector.getreconfImageAfterMaskBLUE().empty()) {
            sensor_msgs::ImagePtr imageMsgAfterMaskBLUE = cv_bridge::CvImage(std_msgs::Header(), "mono8", drum_detector.getreconfImageAfterMaskBLUE()).toImageMsg();
            imagePublisherAfterMaskBLUE.publish(imageMsgAfterMaskBLUE);
        }

        if (!drum_detector.getreconfImageAfterColorEnhancement().empty()) {
            sensor_msgs::ImagePtr imageMsgAfterColorEnhancement = cv_bridge::CvImage(std_msgs::Header(), "mono8", drum_detector.getreconfImageAfterColorEnhancement()).toImageMsg();
            imagePublisherAfterColorEnhancement.publish(imageMsgAfterColorEnhancement);
        }

        if (!drum_detector.getreconfmaskedImage_red().empty()) {
            sensor_msgs::ImagePtr imageMsgImage_red = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drum_detector.getreconfmaskedImage_red()).toImageMsg();
            imagePublisherImage_red.publish(imageMsgImage_red);
        }

        if (!drum_detector.getreconfmaskedImage_blue().empty()) {
            sensor_msgs::ImagePtr imageMsgImage_blue = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drum_detector.getreconfmaskedImage_blue()).toImageMsg();
            imagePublisherImage_blue.publish(imageMsgImage_blue);
        }
    }

public:

    DrumPublisher(const std::string& inputImageTopic, bool enableWindows) : AbstractImageConverter(inputImageTopic), windowsEnabled(enableWindows) {

        image_transport::ImageTransport it(nodeHandle);
        imagePublisher = it.advertise("/CamDrums/image", 100);

        imagePublisherAfterMorphology = it.advertise("/CamDrums/imageAfterMorphology", 100);
        imagePublisherAfterMask = it.advertise("/CamDrums/imageAfterMask", 100);
        imagePublisherAfterMaskRED = it.advertise("/CamDrums/imageAfterMaskRED", 100);
        imagePublisherAfterMaskBLUE = it.advertise("/CamDrums/imageAfterMaskBLUE", 100);
        imagePublisherAfterColorEnhancement = it.advertise("/CamDrums/imageAfterColorEnhancement", 100);
        imagePublisherImage_red = it.advertise("/CamDrums/Image_red", 100);
        imagePublisherImage_blue = it.advertise("/CamDrums/Image_blue", 100);

        /**
         * Tell the master that we are going to be publishing a message
         * of type auv_common/OptionalPoint2D on the topic DRUMS_MAT_PUBLISH_TOPIC_BOTTOM ("/drums/mat/cam_bottom").
         * This lets the master tell any nodes listening on DRUMS_MAT_PUBLISH_TOPIC_BOTTOM
         * that we are going to publish data on that topic.
         */

        drumPublisher = nodeHandle.advertise<auv_common::DrumsCoordinates>(DRUMS_DRUM_PUBLISH_TOPIC, 100);
        if (windowsEnabled)
            cv::namedWindow(OPENCV_WINDOW_DRUM, CV_WINDOW_AUTOSIZE);
    }

    ~DrumPublisher()
    {
        if (windowsEnabled)
            cv::destroyWindow(OPENCV_WINDOW_DRUM);
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
    ros::NodeHandle nodeHandle(DRUMS_LOCATOR_NODE_NAME);

    bool windowsEnabled;
    nodeHandle.param(ENABLE_WINDOWS_PARAM, windowsEnabled, false);

    dynamic_reconfigure::Server<auv_vision::DrumsLocatorConfig> reconfigurationServer;

    dynamic_reconfigure::Server<auv_vision::DrumsLocatorConfig>::CallbackType f;
    f = boost::bind(&reconfigureDrums, _1, _2);
    reconfigurationServer.setCallback(f);

    MatPublisherFrontCam matPublisherFrontCam(CAMERA_FRONT_TOPIC, windowsEnabled);
    MatPublisherBottomCam matPublisherBottomCam(CAMERA_BOTTOM_TOPIC, windowsEnabled);
    DrumPublisherFrontCam drumPublisherFrontCam(CAMERA_FRONT_TOPIC, windowsEnabled);

    DrumPublisher drumPublisher(CAMERA_BOTTOM_TOPIC, windowsEnabled);

    ros::spin();

    return 0;
}
