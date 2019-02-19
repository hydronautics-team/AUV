#ifndef AUV_VISION_GATEDETECTOR_H
#define AUV_VISION_GATEDETECTOR_H

#include "GateDescriptor.h"

#define CUDA_ENABLED 0

class GateDetector {

private:

    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst);

    void detectVerticalLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines);

    float getLineSlope(const cv::Vec4f& line);

    float getLength(const cv::Vec4f& line);

    float getDistance(float x1, float y1, float x2, float y2);

    cv::Point2f getProjection(const cv::Vec4f& line, const cv::Point2f& point);

    void meanShift(const cv::Mat& src, cv::Mat& dst);

    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);

    void morphology(const cv::Mat& src, cv::Mat& dst);

public:

    GateDetector() = default;
    ~GateDetector() = default;

    GateDetector& operator=(const GateDetector& other) = default;

    GateDescriptor detect(const cv::Mat& src, bool withPreprocess);

};

#endif //AUV_VISION_GATEDETECTOR_H
