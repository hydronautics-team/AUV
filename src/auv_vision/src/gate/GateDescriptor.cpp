#include "../../include/gate/GateDescriptor.h"

GateDescriptor GateDescriptor::noGates() { return GateDescriptor(false, std::vector<cv::Point2f>(4)); }

GateDescriptor GateDescriptor::create(const std::vector<cv::Point2f>& corners) { return GateDescriptor(true, corners); }

GateDescriptor::GateDescriptor(bool gates, const std::vector<cv::Point2f>& corners) {
    this->gates = gates;
    if (gates)
        this->corners = corners;
}

GateDescriptor::GateDescriptor(const GateDescriptor &other) {
    this->gates = other.gates;
    this->corners = other.corners;
}

GateDescriptor& GateDescriptor::operator=(const GateDescriptor &other) {
    if (this != &other) {
        this->gates = other.gates;
        this->corners = other.corners;
    }
    return *this;
}

bool GateDescriptor::hasGate() { return gates; }

std::vector<cv::Point2f> GateDescriptor::getCorners() { return corners; }

cv::Point2f GateDescriptor::getCenter() {
    if (!gates)
        return cv::Point2f(0.0, 0.0);
    cv::Moments moments = cv::moments(corners, false);
    return cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
}

cv::Rect GateDescriptor::getBoundingRect() {
    if (!gates)
        return cv::Rect(0.0, 0.0, 0.0, 0.0);
    return cv::boundingRect(corners);
}
