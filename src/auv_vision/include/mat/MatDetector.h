#ifndef AUV_VISION_MATDETECTOR_H
#define AUV_VISION_MATDETECTOR_H

#include <opencv2/opencv.hpp>

#include "../../include/mat/MatDescriptor.h"

class MatDetector {

private:

    float lower_green_H = 78;
    float lower_green_S = 95;
    float lower_green_V = 0;

    float higher_green_H = 180;
    float higher_green_S = 255;
    float higher_green_V = 146;


    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst); // Image filter
    void extractGreenColour(const cv::Mat& src, cv::Mat& dst); // For colour filtering
    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);
    void thresholdAndContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours);


    void morphology(const cv::Mat& src, cv::Mat& dst);
    void meanShift(const cv::Mat& src, cv::Mat& dst);

public:

    float setLowerGreenH() const;
    void setLowerGreenH(float lower_green_H);

    float setHigherGreenH() const;
    void setHigherGreenH(float higher_green_H);

    float setLowerGreenS() const;
    void setLowerGreenS(float lower_green_S);

    float setHigherGreenS() const;
    void setHigherGreenS(float higher_green_S);

    float setLowerGreenV() const;
    void setLowerGreenV(float lower_green_V);

    float setHigherGreenV() const;
    void setHigherGreenV(float higher_green_V);

    cv::Mat getimageAfterContourDetection();
    cv::Mat getLinesImage();


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

class FrontCameraDrumDetector {

private:

    cv::Mat1b getMask(const cv::Mat& src);
    std::vector<std::vector<cv::Point>> getDrumContour(const cv::Mat1b& image);
    float getDrumCoordinates(const std::vector<std::vector<cv::Point>>& contours);

public:

    FrontCameraDrumDetector() = default;
    ~FrontCameraDrumDetector() = default;
    FrontCameraDrumDetector& operator=(const FrontCameraDrumDetector& other) = default;

    FrontCameraDrumDescriptor detect(const cv::Mat& src, cv::Mat& image);

    cv::Mat getimageAfterContourDetectionDrum();
    cv::Mat getimageAfterMaskDrum();

};

/**************************************************************************/

class MatDetectorBottomCamera {

private:

    float length_threshold = 60;
    float distance_threshold = 6;
    float min_angle_criteria = 8;
    float max_angle_criteria = 83;

    void detectLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines);
    std::vector<std::vector<float>> drawAndSortLines(cv::Mat& image, std::vector<cv::Vec4f>& lines);
    float getLineSlope(const cv::Vec4f& line);
    std::vector<cv::Vec4f> findHorizontalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines);
    std::vector<cv::Vec4f> findVerticalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines);
    float getLength(const cv::Vec4f& line);
    float getDistance(float x1, float y1, float x2, float y2);

public:

    float setLengthThreshold() const;
    void setLengthThreshold(float length_threshold);

    float setDistanceThreshold() const;
    void setDistanceThreshold(float distance_threshold);

    float setMinAngleCriteria() const;
    void setMinAngleCriteria(float min_angle_criteria);

    float setMaxAngleCriteria() const;
    void setMaxAngleCriteria(float max_angle_criteria);

    cv::Mat getimageWithAllLines();

    MatDetectorBottomCamera() = default;
    ~MatDetectorBottomCamera() = default;
    MatDetectorBottomCamera& operator=(const MatDetectorBottomCamera& other) = default;

    MatDescriptorBottomCamera detect(const cv::Mat& src, cv::Mat& image);

};


#endif //AUV_VISION_MATDETECTOR_H
