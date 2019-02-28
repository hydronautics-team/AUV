#ifndef AUV_VISION_GATEDETECTOR_H
#define AUV_VISION_GATEDETECTOR_H

#include <opencv2/ximgproc.hpp>
#include "GateDescriptor.h"


class GateDetector {

private:

    float verticalSlope = 10.0f;
    float horizontalSlope = 10.0f;
    float lengthRelation = 0.15f;
    float mergingLineDistanceHorizontal = 8.0f;
    float mergingLineDistanceVertical = 5.0f;

    float overlapThreshold = 3.0f;
    float distXThreshold = 17.0f;
    float distYThreshold = 17.0f;
    float sidesRelationThreshold = 0.7f;
    float angleDiffThreshold = 7.0f;
    float areaFrameRelationThreshold = 0.05f;


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

    GateDescriptor findBestByQuality(const std::vector<cv::Vec4f>& verticalLines,
                                     const std::vector<cv::Vec4f>& horizontalLines, long frameArea);

    void updateFLD();

public:

    GateDetector() = default;
    ~GateDetector() = default;

    GateDetector& operator=(const GateDetector& other) = default;

    GateDescriptor detect(const cv::Mat& src);

    void setVerticalSlope(float verticalSlope);

    void setHorizontalSlope(float horizontalSlope);

    void setLengthRelation(float lengthRelation);

    void setMergingLineDistanceHorizontal(float mergingLineDistanceHorizontal);

    void setMergingLineDistanceVertical(float mergingLineDistanceVertical);

    void setOverlapThreshold(float overlapThreshold);

    void setDistXThreshold(float distXThreshold);

    void setDistYThreshold(float distYThreshold);

    void setSidesRelationThreshold(float sidesRelationThreshold);

    void setAngleDiffThreshold(float angleDiffThreshold);

    void setAreaFrameRelationThreshold(float areaFrameRelationThreshold);

    void setLength_threshold(int length_threshold);

    void setDistance_threshold(float distance_threshold);

    void setCanny_th1(double canny_th1);

    void setCanny_th2(double canny_th2);

    void setCanny_aperture_size(int canny_aperture_size);

};

#endif //AUV_VISION_GATEDETECTOR_H
