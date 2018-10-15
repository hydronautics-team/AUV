#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "../../include/gate/GateDetector.h"
#include "../../include/gate/GateDescriptor.h"
#include "../../include/util/ImgprocPipeline.h"


void GateDetector::defaultPreprocess(const cv::Mat &src, cv::Mat &dst) {
    // TODO Define which pre-processing methods should be in common scope
    dst = createPipeline(src, false)
            //.apply(std::bind(&GateDetector::meanShift, this, std::placeholders::_1, std::placeholders::_2), "MeanShift")
            .apply(std::bind(&GateDetector::extractValueChannel, this, std::placeholders::_1, std::placeholders::_2), "Value channel")
            .apply(std::bind(&GateDetector::morphology, this, std::placeholders::_1, std::placeholders::_2), "Closing")
            .getImage();
}

void GateDetector::meanShift(const cv::Mat &src, cv::Mat &dst) {

// TODO Fix undefined reference errors
#if CUDA_ENABLED == 0
    cv::pyrMeanShiftFiltering(src, dst, 30, 5);
#else
    cv::cuda::GpuMat gpuSrc, gpuDest;
    gpuSrc.upload(src);
    cv::cuda::cvtColor(gpuSrc, gpuSrc, CV_BGR2BGRA);
    cv::cuda::meanShiftFiltering(gpuSrc, gpuDest, 30, 5);
    gpuDest.download(dst);
#endif

}

void GateDetector::extractValueChannel(const cv::Mat &src, cv::Mat &dst) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    dst = hsvChannels[2];
}

void GateDetector::morphology(const cv::Mat &src, cv::Mat &dst) {
    int size = 1;
    cv::Mat element = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(2*size+1, 2*size+1));
    cv::morphologyEx(src, dst, cv::MORPH_CLOSE, element);
}

void GateDetector::detectVerticalLines(const cv::Mat &image, std::vector<cv::Vec4f> &lines) {

    cv::Ptr<cv::LineSegmentDetector> detector = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> allLines;
    detector->detect(image, allLines);

    double maxLength = 0;
    std::vector<cv::Vec4f> filteredLines;
    for (int i = 0; i < allLines.size(); i++) {
        if (getLineSlope(allLines[i]) < 10.0f) {
            filteredLines.push_back(allLines[i]);
            double length = getLength(allLines[i]);
            if (length > maxLength)
                maxLength = length;
        }
    }

    for (int i = 0; i < filteredLines.size(); i++) {
        double length = getLength(filteredLines[i]);
        if (length >= 0.1*maxLength)
            lines.push_back(filteredLines[i]);
    }
}

float GateDetector::getLineSlope(const cv::Vec4f &line) {
    if (line[1] == line[3])
        return INFINITY;
    float tan = std::abs(line[0] - line[2]) / std::abs(line[1] - line[3]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

float GateDetector::getLength(const cv::Vec4f &line) {
    return std::sqrt((line[2]-line[0])*(line[2]-line[0]) + (line[3]-line[1])*(line[3]-line[1]));
}

float GateDetector::getDistance(float x1, float y1, float x2, float y2) {
    return std::sqrt((x1 - x2)*(x1 - x2) - (y1 - y2)*(y1 - y2));
}

cv::Point2f GateDetector::getProjection(const cv::Vec4f &line, const cv::Point2f &point) {
    cv::Vec2f vec1 = { line[2] - line[0], line[3] - line[1] };
    cv::Vec2f vec2 = { point.x - line[0], point.y - line[1] };

    float vec1Squared = vec1[0]*vec1[0] + vec1[1]*vec1[1];
    float dot = vec1[0]*vec2[0] + vec1[1]*vec2[1];
    cv::Vec2f projection = { vec1[0] * (dot / vec1Squared), vec1[1] * (dot / vec1Squared) };

    return cv::Point2f(line[0] + projection[0], line[1] + projection[1]);
}

GateDescriptor GateDetector::detect(const cv::Mat &src, bool withPreprocess) {

    cv::Mat image;
    if (withPreprocess)
        defaultPreprocess(src, image);
    else
        image = src;

    // Step 1: find all vertical lines on image
    std::vector<cv::Vec4f> verticalLines;
    detectVerticalLines(image, verticalLines);
    if (verticalLines.empty())
        return GateDescriptor::noGates();

    // Step 2: gather all ending points from detected lines
    std::vector<cv::Point2f> allPoints;
    for (int i = 0; i < verticalLines.size(); i++) {
        allPoints.emplace_back(verticalLines[i][0], verticalLines[i][1]);
        allPoints.emplace_back(verticalLines[i][2], verticalLines[i][3]);
    }

    // Step 3: sort points by X value
    std::sort(allPoints.begin(), allPoints.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) -> bool { return a.x > b.x; });

    // Step 4: gather points that lie on one vertical line - it is equals to mergin original detected lines
    cv::Point2f currentPoint = allPoints[0];
    std::vector<std::vector<cv::Point2f>> pointLines;
    std::vector<cv::Point2f> currentPointLine;
    currentPointLine.push_back(currentPoint);
    for (int i = 1; i < allPoints.size(); i++) {

        if (std::abs(allPoints[i].x - currentPoint.x) > 22.0f) {
            pointLines.push_back(currentPointLine);
            currentPointLine.clear();
        }
        currentPointLine.push_back(allPoints[i]);

        currentPoint = allPoints[i];
    }
    pointLines.emplace_back(currentPointLine);
    currentPointLine.clear();

    std::vector<cv::Vec4f> mergedLines;
    for (int i = 0; i < pointLines.size(); i++) {
        // TODO: Use line fitting
        auto edges  = std::minmax_element(pointLines[i].begin(), pointLines[i].end(),
                                          [](const cv::Point2f& a, const cv::Point2f& b) {
                                              return a.y > b.y;
                                          });

        mergedLines.push_back({(*(edges.first)).x, (*(edges.first)).y, (*(edges.second)).x, (*(edges.second)).y});
    }


    // Step 5: find two longest lines
    std::sort(mergedLines.begin(), mergedLines.end(), [this](const cv::Vec4f& a, const cv::Vec4f& b) {
        return getLength(a) > getLength(b);
    });
    cv::Vec4f line1 = mergedLines[0];
    cv::Vec4f line2 = mergedLines[1];

    // Step 6: check length relation between lines and relation between possible gates' sides
    float verticalRelation = getLength(line2) / getLength(line1);
    float sidesRelation = getDistance(line1[0], line1[1], line2[0], line2[1]) / getLength(line2);
    if (verticalRelation < 0.4f || sidesRelation < 0.4f || sidesRelation > 1.51f)
        return GateDescriptor::noGates();

    // Step 7: find corners of the gates
    cv::Vec4f leftLine, rightLine;
    if (line1[2] < line2[2]) {
        leftLine = line1;
        rightLine = line2;
    } else {
        leftLine = line2;
        rightLine = line1;
    }

    //cv::Point2f topLeft(std::min(line1[2], line2[2]), std::min(line1[3], line2[3]));
    //cv::Point2f bottomRight(std::max(line1[0], line2[0]), std::min(line1[1], line2[1]));

    cv::Point2f topLeft, topRight, bottomRight, bottomLeft;

    if (leftLine[3] < rightLine[3]) {
        topLeft = cv::Point2f(leftLine[2], leftLine[3]);
        topRight = getProjection(rightLine, topLeft);
    } else {
        topRight = cv::Point2f(rightLine[2], rightLine[3]);
        topLeft = getProjection(leftLine, topRight);
    }

    if (rightLine[1] < leftLine[1]) {
        bottomRight = cv::Point2f(rightLine[0], rightLine[1]);
        bottomLeft = getProjection(leftLine, bottomRight);
    } else {
        bottomLeft = cv::Point2f(leftLine[0], leftLine[1]);
        bottomRight = getProjection(rightLine, bottomLeft);
    }

    return GateDescriptor::create({topLeft, topRight, bottomRight, bottomLeft});
}