#ifndef PROJECT_VERTICALGATEDETECTOR_H
#define PROJECT_VERTICALGATEDETECTOR_H

#include "IGateDetector.h"

class VerticalGateDetector : public IGateDetector {

private:

    GateDescriptor findBestByQuality(const std::vector<cv::Vec4f>& verticalLines,
                                     long frameWidth, long frameHeight);

    float parallelRelationThreshold = 0.7f;
    float horizontalSlopeThreshold = 10.0f;
    float orthogonalRelationErrorThreshold = 0.3f;
    float areaFrameRelationThreshold = 0.05f;

    void setParallelRelationThreshold(float parallelRelationThreshold);

    void setHorizontalSlopeThreshold(float horizontalSlopeThreshold);

    void setOrthogonalRelationErrorThreshold(float orthogonalRelationErrorThreshold);

    void setAreaFrameRelationThreshold(float areaFrameRelationThreshold);

public:

    VerticalGateDetector(float horizontalToVerticalRelation);
    ~VerticalGateDetector() = default;

    void reconfigure(auv_vision::GateLocatorConfig& config, uint32_t level) override;

    GateDescriptor detect(const cv::Mat& src) override;

};

#endif //PROJECT_VERTICALGATEDETECTOR_H
