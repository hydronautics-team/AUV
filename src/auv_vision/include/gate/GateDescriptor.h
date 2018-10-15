#ifndef AUV_VISION_GATEDESCRIPTOR_H
#define AUV_VISION_GATEDESCRIPTOR_H

#include <opencv2/opencv.hpp>
#include "../msg/gate_msg.h"

class GateDescriptor {

private:

    bool gates;
    std::vector<cv::Point2f> corners;

    GateDescriptor(bool gates, const std::vector<cv::Point2f>& corners);

public:

    static GateDescriptor noGates();
    static GateDescriptor create(const std::vector<cv::Point2f>& corners);
    static GateDescriptor fromMsg(const auv_vision::gate_msg& msg);

    GateDescriptor(const GateDescriptor& other);
    ~GateDescriptor() = default;

    GateDescriptor& operator=(const GateDescriptor& other);

    bool hasGate();
    std::vector<cv::Point2f> getCorners();
    cv::Point2f getCenter();
    cv::Rect getBoundingRect();

    auv_vision::gate_msg toMsg();

};

#endif //AUV_VISION_GATEDESCRIPTOR_H
