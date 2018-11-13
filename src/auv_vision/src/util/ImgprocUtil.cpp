#include "../../include/util/ImgprocUtil.h"

cv::Point2f convertToCentralCoordinates(const cv::Point2f& point, int imageWidth, int imageHeight) {

    int centerX = imageWidth / 2;
    int centerY = imageHeight / 2;

    return cv::Point2f(point.x - centerX, centerY - point.y);
}