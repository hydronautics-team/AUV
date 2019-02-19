#ifndef AUV_VISION_DRUMDETECTOR_H
#define AUV_VISION_DRUMDETECTOR_H

#include <opencv2/opencv.hpp>
#include "DrumDescriptor.h"

class DrumDetector {

private:

    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst); // Image filter
    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);
    void detectContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours);
    std::vector<cv::Vec3f> findCircles(const cv::Mat& src, cv::Mat& dst);

    void morphology(const cv::Mat& src, cv::Mat& dst);
    void meanShift(const cv::Mat& src, cv::Mat& dst);

public:

    bool isRed(const cv::Mat& src, const cv::Vec3f circle);
    bool isBlue(const cv::Mat& src, const cv::Vec3f circle);

    DrumDetector() = default;
    ~DrumDetector() = default;
    DrumDetector& operator=(const DrumDetector& other) = default;

    DrumDescriptor detect(const cv::Mat& src, bool withPreprocess);

};

#endif //AUV_VISION_DRUMDETECTOR_H
