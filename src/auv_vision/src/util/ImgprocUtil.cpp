#include "../../include/util/ImgprocUtil.h"

cv::Point2f convertToCentralCoordinates(const cv::Point2f& point, int imageWidth, int imageHeight) {

    int centerX = imageWidth / 2;
    int centerY = imageHeight / 2;

    return cv::Point2f(point.x - centerX, centerY - point.y);
}

cv::Point2f computeIntersect(cv::Vec4f a, cv::Vec4f b) {
    float x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
    float denom = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));

    if (denom != 0)
    {
        cv::Point2f pt;
        pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
        pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
        return pt;
    }
    else {
        std::cerr<<"Lines are parallel bro"<<std::endl;
        return cv::Point2f(-1, -1);
    }
}
