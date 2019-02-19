#ifndef AUV_VISION_IMGPROCUTIL_H
#define AUV_VISION_IMGPROCUTIL_H

#include <opencv2/opencv.hpp>

/**
 * Set of useful methods for image processing.
 */

/**
 * Converts point coordinates from top-left coordinate system
 * to central coordinate system.
 */
cv::Point2f convertToCentralCoordinates(const cv::Point2f& point, int imageWidth, int imageHeight);

cv::Point2f computeIntersect(cv::Vec4f a, cv::Vec4f b);

#endif //AUV_VISION_IMGPROCUTIL_H
