#include "../../include/gate/GateDescriptor.h"

GateDescriptor GateDescriptor::noGates() { return GateDescriptor(false, std::vector<cv::Point2f>(4)); }

GateDescriptor GateDescriptor::create(const std::vector<cv::Point2f>& corners) { return GateDescriptor(true, corners); }

GateDescriptor GateDescriptor::fromMsg(const auv_vision::gate_msg &msg) {
    if (msg.x1 == msg.x2 == msg.x3 == msg.x4 == msg.y1 == msg.y2 == msg.y3 == msg.x4 == 0)
        return GateDescriptor::noGates();
    return GateDescriptor(true, {
        cv::Point2f(msg.x1, msg.y1),
        cv::Point2f(msg.x2, msg.y2),
        cv::Point2f(msg.x3, msg.y3),
        cv::Point2f(msg.x4, msg.y4)
    });
}

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

auv_vision::gate_msg GateDescriptor::toMsg() {

    auv_vision::gate_msg msg;

    if (gates) {
        msg.x1 = corners[0].x;
        msg.y1 = corners[0].y;
        msg.x2 = corners[1].x;
        msg.y2 = corners[1].y;
        msg.x3 = corners[2].x;
        msg.y3 = corners[2].y;
        msg.x4 = corners[3].x;
        msg.y4 = corners[3].y;
    } else
        msg.x1 = msg.x2 = msg.x3 = msg.x4 = msg.y1 = msg.y2 = msg.y3 = msg.y4 = 0;

    return msg;
}
