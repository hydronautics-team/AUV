#ifndef PROJECT_ANGLEGATEDETECTOR_H
#define PROJECT_ANGLEGATEDETECTOR_H

#include "IGateDetector.h"

class AngleGateDetector : public IGateDetector {

private:

    float overlapThreshold = 3.0f;
    float distXThreshold = 17.0f;
    float distYThreshold = 17.0f;
    float sidesRelationErrorThreshold = 0.3f;
    float angleDiffThreshold = 7.0f;
    float areaFrameRelationThreshold = 0.05f;
    float horizontalPositionRatioThreshold = 0.45f;

    GateDescriptor findBestByQuality(const std::vector<cv::Vec4f>& verticalLines,
                                     const std::vector<cv::Vec4f>& horizontalLines,
                                     long frameWidth, long frameHeight);

    void setOverlapThreshold(float overlapThreshold);

    void setDistXThreshold(float distXThreshold);

    void setDistYThreshold(float distYThreshold);

    void setSidesRelationErrorThreshold(float sidesRelationThreshold);

    void setAngleDiffThreshold(float angleDiffThreshold);

    void setAreaFrameRelationThreshold(float areaFrameRelationThreshold);

    void setHorizontalPositionRatioThreshold(float horizontalPositionRatioThreshold);


public:

    AngleGateDetector(float horizontalToVerticalRelation);
    ~AngleGateDetector() = default;

    void reconfigure(auv_vision::GateLocatorConfig& config, uint32_t level) override;

    GateDescriptor detect(const cv::Mat& src) override;

};

#endif //PROJECT_ANGLEGATEDETECTOR_H
