#ifndef AUV_VISION_GATEDETECTOR_H
#define AUV_VISION_GATEDETECTOR_H

#include "GateDescriptor.h"

#define CUDA_ENABLED 0

class GateDetector {

private:

    float verticalSlope = 10.0f;
    float verticalLengthRelation = 0.1f;
    float mergingLineDistance = 22.0f;
    float angleQualityThreshold = 5.0f;
    float totalQualityThreshold = 1.5f;

    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst);

    void detectVerticalLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines);

    float getLineSlope(const cv::Vec4f& line);

    float getLength(const cv::Vec4f& line);

    float getDistance(float x1, float y1, float x2, float y2);

    cv::Point2f getProjection(const cv::Vec4f& line, const cv::Point2f& point);

    void meanShift(const cv::Mat& src, cv::Mat& dst);

    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);

    void morphology(const cv::Mat& src, cv::Mat& dst);

    float getAngle(float x1, float y1, float x2, float y2, float x3, float y3);

public:

    GateDetector() = default;
    ~GateDetector() = default;

    GateDetector& operator=(const GateDetector& other) = default;

    GateDescriptor detect(const cv::Mat& src, bool withPreprocess);

    float getVerticalSlope() const;

    void setVerticalSlope(float verticalSlope);

    float getVerticalLengthRelation() const;

    void setVerticalLengthRelation(float verticalLengthRelation);

    float getMergingLineDistance() const;

    void setMergingLineDistance(float mergingLineDistance);

    float getAngleQualityThreshold() const;

    void setAngleQualityThreshold(float angleQualityThreshold);

    float getTotalQualityThreshold() const;

    void setTotalQualityThreshold(float totalQualityThreshold);

};

#endif //AUV_VISION_GATEDETECTOR_H
