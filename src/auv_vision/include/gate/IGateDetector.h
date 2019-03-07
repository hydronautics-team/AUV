#ifndef AUV_VISION_IGATEDETECTOR_H
#define AUV_VISION_IGATEDETECTOR_H

#include <opencv2/ximgproc.hpp>
#include <auv_vision/GateLocatorConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GateDescriptor.h"


class IGateDetector {

protected:

    float horizontalToVerticalRelation;

    float verticalSlope = 10.0f;
    float horizontalSlope = 10.0f;
    float lengthRelation = 0.15f;
    float mergingLineDistanceHorizontal = 10.0f;
    float mergingLineDistanceVertical = 5.0f;

    /** FLD settings */
    int length_threshold = 10;
    float distance_threshold = 15.0f;
    double canny_th1 = 10.0;
    double canny_th2 = 10.0;
    int canny_aperture_size = 7;
    bool do_merge = false;

    cv::Ptr<cv::ximgproc::FastLineDetector> fld =
            cv::ximgproc::createFastLineDetector(length_threshold,
                                                 distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);

    image_transport::Publisher imagePublisher;

    void detectLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines);

    void filterVerticalAndHorizontal(const std::vector<cv::Vec4f>& lines,
                                     std::vector<cv::Vec4f>& vertical, std::vector<cv::Vec4f>& horizontal);

    float getLineSlopeVertical(const cv::Vec4f& line);

    float getLineSlopeHorizontal(const cv::Vec4f& line);

    float getLength(const cv::Vec4f& line);

    bool isAbleToVerticalMerge(const cv::Vec4f &line1, const cv::Vec4f &line2);

    void mergeVerticalLines(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines);

    void mergeY(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines);

    cv::Vec4f createVerticalLine(const std::vector<cv::Vec4f> &lines);

    void updateFLD();

    void setVerticalSlope(float verticalSlope);

    void setHorizontalSlope(float horizontalSlope);

    void setLengthRelation(float lengthRelation);

    void setMergingLineDistanceHorizontal(float mergingLineDistanceHorizontal);

    void setMergingLineDistanceVertical(float mergingLineDistanceVertical);

    void setLength_threshold(int length_threshold);

    void setDistance_threshold(float distance_threshold);

    void setCanny_th1(double canny_th1);

    void setCanny_th2(double canny_th2);

    void setCanny_aperture_size(int canny_aperture_size);

public:

    IGateDetector(float horizontalToVerticalRelation);
    ~IGateDetector() = default;

    void setPublisher(const ros::NodeHandle& nh);

    virtual void reconfigure(auv_vision::GateLocatorConfig& config, uint32_t level);

    virtual GateDescriptor detect(const cv::Mat& src) = 0;
};


#endif //PROJECT_IGATEDETECTOR_H
