#ifndef AUV_VISION_MATDETECTOR_H
#define AUV_VISION_MATDETECTOR_H

#include <opencv2/opencv.hpp>

#include "../../include/mat/MatDescriptor.h"

class MatDetector {

private:

    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst); // Image filter
    void extractGreenColour(const cv::Mat& src, cv::Mat& dst); // For colour filtering
    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);
    void thresholdAndContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours);


    void morphology(const cv::Mat& src, cv::Mat& dst);
    void meanShift(const cv::Mat& src, cv::Mat& dst);

public:

    void detectContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours, bool withPreprocess);

    MatDetector() = default;
    ~MatDetector() = default;
    MatDetector& operator=(const MatDetector& other) = default;

};

/**************************************************************************/

class MatDetectorFrontCamera {

private:

    bool getMatContour(std::vector<std::vector<cv::Point>>& contours, const cv::Mat& src);

public:

    MatDetectorFrontCamera() = default;
    ~MatDetectorFrontCamera() = default;
    MatDetectorFrontCamera& operator=(const MatDetectorFrontCamera& other) = default;

    MatDescriptorFrontCamera detect(const cv::Mat& src, cv::Mat& image, std::vector<std::vector<cv::Point>>& contours);

};

/**************************************************************************/

class MatDetectorBottomCamera {

private:

    void detectLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines);
    std::vector<std::vector<float>> drawAndSortLines(cv::Mat& image, std::vector<cv::Vec4f>& lines);
    float getLineSlope(const cv::Vec4f& line);
    std::vector<cv::Vec4f> findHorizontalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines);
    std::vector<cv::Vec4f> findVerticalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines);
    float getLength(const cv::Vec4f& line);
    float getDistance(float x1, float y1, float x2, float y2);

public:

    MatDetectorBottomCamera() = default;
    ~MatDetectorBottomCamera() = default;
    MatDetectorBottomCamera& operator=(const MatDetectorBottomCamera& other) = default;

    MatDescriptorBottomCamera detect(const cv::Mat& src, cv::Mat& image);

};


#endif //AUV_VISION_MATDETECTOR_H
