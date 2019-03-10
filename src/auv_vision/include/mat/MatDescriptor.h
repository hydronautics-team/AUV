#ifndef AUV_VISION_MATDESCRIPTOR_H
#define AUV_VISION_MATDESCRIPTOR_H


#include <opencv2/opencv.hpp>

class MatDescriptorFrontCamera {

private:

    bool mat;
    std::vector<std::vector<cv::Point>> contour;

    MatDescriptorFrontCamera(bool mat, const std::vector<std::vector<cv::Point>>& contour);

public:

    static MatDescriptorFrontCamera noMat();
    static MatDescriptorFrontCamera create(const std::vector<std::vector<cv::Point>>& contour);

    MatDescriptorFrontCamera(const MatDescriptorFrontCamera& other); /// Copy constructor
    ~MatDescriptorFrontCamera() = default; /// Destructor
    MatDescriptorFrontCamera& operator=(const MatDescriptorFrontCamera& other); /// Copy assignment

    bool hasMat();
    cv::Point2f getCenter();
    std::vector<std::vector<cv::Point>> getContour();
    cv::Rect getBoundingRect();

};

class FrontCameraDrumDescriptor {

private:

    bool drum;
    std::vector<std::vector<cv::Point>> contour;

    FrontCameraDrumDescriptor(bool drum, const std::vector<std::vector<cv::Point>>& contour);

public:

    static FrontCameraDrumDescriptor noDrum();
    static FrontCameraDrumDescriptor create(const std::vector<std::vector<cv::Point>>& contour);

    FrontCameraDrumDescriptor(const FrontCameraDrumDescriptor& other); /// Copy constructor
    ~FrontCameraDrumDescriptor() = default; /// Destructor
    FrontCameraDrumDescriptor& operator=(const FrontCameraDrumDescriptor& other); /// Copy assignment

    bool hasDrum();
    cv::Point2f getCenter();
    std::vector<std::vector<cv::Point>> getContour();
    cv::Rect getBoundingRect();

};

class MatDescriptorBottomCamera {

private:

    //bool mat;
    bool horizontal_lines, vertical_lines;
    std::vector<cv::Vec4f> horizontalLines;
    std::vector<cv::Vec4f> verticalLines;

    MatDescriptorBottomCamera(bool horizontal_lines, bool vertical_lines, const std::vector<cv::Vec4f>& horizontalLines, const std::vector<cv::Vec4f>& verticalLines);

public:

    static MatDescriptorBottomCamera noLines();
    static MatDescriptorBottomCamera create(const std::vector<cv::Vec4f>& horizontalLines, const std::vector<cv::Vec4f>& verticalLines);

    MatDescriptorBottomCamera(const MatDescriptorBottomCamera& other);
    ~MatDescriptorBottomCamera() = default;
    MatDescriptorBottomCamera& operator=(const MatDescriptorBottomCamera& other);

    bool hasHorizontalLines();
    bool hasVerticalLines();

    std::vector<cv::Vec4f> getHorizontalLines();
    std::vector<cv::Vec4f> getVerticalLines();

    float getIntersectionWithHorizontal(const cv::Mat& src, cv::Vec4f& horizontalLine);
    float getIntersectionWithVertical(const cv::Mat& src, cv::Vec4f& verticalLine);
};

#endif //AUV_VISION_MATDESCRIPTOR_H
