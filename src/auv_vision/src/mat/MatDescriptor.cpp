#include "../../include/mat/MatDescriptor.h"
#include "../../include/util/ImgprocUtil.h"

MatDescriptorFrontCamera MatDescriptorFrontCamera::noMat() {return MatDescriptorFrontCamera(false, std::vector<std::vector<cv::Point>> ());}

MatDescriptorFrontCamera MatDescriptorFrontCamera::create(const std::vector<std::vector<cv::Point>>& contour) {return MatDescriptorFrontCamera(true, contour);}

MatDescriptorFrontCamera::MatDescriptorFrontCamera(bool mat, const std::vector<std::vector<cv::Point>>& contour) {
    this->mat = mat;
    if (mat)
        this->contour = contour;
}

MatDescriptorFrontCamera::MatDescriptorFrontCamera(const MatDescriptorFrontCamera &other) {
    this->mat = other.mat;
    this->contour = other.contour;
}

MatDescriptorFrontCamera& MatDescriptorFrontCamera::operator=(const MatDescriptorFrontCamera &other) {
    if (this != &other) {
        this->mat = other.mat;
        this->contour = other.contour;
    }
    return *this;
}

bool MatDescriptorFrontCamera::hasMat() {return mat;}

std::vector<std::vector<cv::Point>> MatDescriptorFrontCamera::getContour() {return contour;}

cv::Point2f MatDescriptorFrontCamera::getCenter() {
    if (!mat)
        return cv::Point2f(0.0, 0.0);
    /// Moments
/*
    /// Get the moment
    cv::Moments mu;
    mu = moments(contour[0], false);

    /// Get the mass center:
    return cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
*/

    cv::Point2f center_of_rect = (cv::boundingRect(contour[0]).br() + cv::boundingRect(contour[0]).tl())*0.5;
    return center_of_rect;

    // https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html - for several contours
}

cv::Rect MatDescriptorFrontCamera::getBoundingRect() {
    if (!mat)
        return cv::Rect(0.0, 0.0, 0.0, 0.0);
    return cv::boundingRect(contour[0]);
}

/********************************************************************/

FrontCameraDrumDescriptor FrontCameraDrumDescriptor::noDrum() {return FrontCameraDrumDescriptor(false, std::vector<std::vector<cv::Point>> ());}

FrontCameraDrumDescriptor FrontCameraDrumDescriptor::create(const std::vector<std::vector<cv::Point>>& contour) {return FrontCameraDrumDescriptor(true, contour);}

FrontCameraDrumDescriptor::FrontCameraDrumDescriptor(bool drum, const std::vector<std::vector<cv::Point>>& contour) {
    this->drum = drum;
    if (drum)
        this->contour = contour;
}

FrontCameraDrumDescriptor::FrontCameraDrumDescriptor(const FrontCameraDrumDescriptor &other) {
    this->drum = other.drum;
    this->contour = other.contour;
}

FrontCameraDrumDescriptor& FrontCameraDrumDescriptor::operator=(const FrontCameraDrumDescriptor &other) {
    if (this != &other) {
        this->drum = other.drum;
        this->contour = other.contour;
    }
    return *this;
}

bool FrontCameraDrumDescriptor::hasDrum() {return drum;}

std::vector<std::vector<cv::Point>> FrontCameraDrumDescriptor::getContour() {return contour;}

cv::Point2f FrontCameraDrumDescriptor::getCenter() {
    if (!drum)
        return cv::Point2f(0.0, 0.0);
    /// Moments
/*
    /// Get the moment
    cv::Moments mu;
    mu = moments(contour[0], false);

    /// Get the mass center:
    return cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
*/

    cv::Point2f center_of_rect = (cv::boundingRect(contour[0]).br() + cv::boundingRect(contour[0]).tl())*0.5;
    return center_of_rect;

    // https://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/moments/moments.html - for several contours
}

cv::Rect FrontCameraDrumDescriptor::getBoundingRect() {
    if (!drum)
        return cv::Rect(0.0, 0.0, 0.0, 0.0);
    return cv::boundingRect(contour[0]);
}

/********************************************************************/

MatDescriptorBottomCamera MatDescriptorBottomCamera::noLines() {
    return MatDescriptorBottomCamera(false, false, std::vector<cv::Vec4f>(), std::vector<cv::Vec4f>());
}

MatDescriptorBottomCamera MatDescriptorBottomCamera::create(const std::vector<cv::Vec4f>& horizontalLines, const std::vector<cv::Vec4f>& verticalLines) {
    if (!horizontalLines.empty() && (!verticalLines.empty())) return MatDescriptorBottomCamera(true, true,  horizontalLines, verticalLines);
    else if (!horizontalLines.empty()) return MatDescriptorBottomCamera(true, false,  horizontalLines, std::vector<cv::Vec4f>());
    else return MatDescriptorBottomCamera(false, true, std::vector<cv::Vec4f>(), verticalLines);
}

MatDescriptorBottomCamera::MatDescriptorBottomCamera(bool horizontal_lines, bool vertical_lines, const std::vector<cv::Vec4f>& horizontalLines, const std::vector<cv::Vec4f>& verticalLines) {
    this->horizontal_lines = horizontal_lines;
    this->vertical_lines = vertical_lines;
    if (horizontal_lines)
        this->horizontalLines = horizontalLines;
    if (vertical_lines)
        this->verticalLines = verticalLines;
}

MatDescriptorBottomCamera::MatDescriptorBottomCamera(const MatDescriptorBottomCamera &other) {
    this->horizontal_lines = other.horizontal_lines;
    this->vertical_lines = other.vertical_lines;
    this->horizontalLines = other.horizontalLines;
    this->verticalLines = other.verticalLines;
}

MatDescriptorBottomCamera& MatDescriptorBottomCamera::operator=(const MatDescriptorBottomCamera &other) {
    if (this != &other) {
        this->horizontal_lines = other.horizontal_lines;
        this->vertical_lines = other.vertical_lines;
        this->horizontalLines = other.horizontalLines;
        this->verticalLines = other.verticalLines;
    }
    return *this;
}

bool MatDescriptorBottomCamera::hasHorizontalLines() {return horizontal_lines;}
bool MatDescriptorBottomCamera::hasVerticalLines() {return vertical_lines;}

std::vector<cv::Vec4f> MatDescriptorBottomCamera::getHorizontalLines() {return horizontalLines;}
std::vector<cv::Vec4f> MatDescriptorBottomCamera::getVerticalLines() {return verticalLines;}

float MatDescriptorBottomCamera::getIntersectionWithHorizontal(const cv::Mat& src, cv::Vec4f& horizontalLine) {
    cv::Vec4i verticalLine_zero(src.size().width/2, src.size().height/2, src.size().width/2, 0);
    cv::Vec4i horizontalLine_zero(src.size().width/2, src.size().height/2, 0, src.size().height/2);

    return convertToCentralCoordinates(computeIntersect(horizontalLine, verticalLine_zero), src.size().width, src.size().height).y;

}

float MatDescriptorBottomCamera::getIntersectionWithVertical(const cv::Mat &src, cv::Vec4f &verticalLine){
    cv::Vec4i verticalLine_zero(src.size().width/2, src.size().height/2, src.size().width/2, 0);
    cv::Vec4i horizontalLine_zero(src.size().width/2, src.size().height/2, 0, src.size().height/2);

    return convertToCentralCoordinates(computeIntersect(verticalLine, horizontalLine_zero), src.size().width, src.size().height).x;
}
