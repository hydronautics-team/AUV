#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

/**
#if CUDA_ENABLED == 0
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
*/

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

    // TODO Fix problems with CUDA dependencies
    cv::pyrMeanShiftFiltering(src, dst, 30, 5);
/**
#if CUDA_ENABLED == 0
    cv::pyrMeanShiftFiltering(src, dst, 30, 5);
#else
    cv::cuda::GpuMat gpuSrc, gpuDest;
    gpuSrc.upload(src);
    cv::cuda::cvtColor(gpuSrc, gpuSrc, CV_BGR2BGRA);
    cv::cuda::meanShiftFiltering(gpuSrc, gpuDest, 30, 5);
    gpuDest.download(dst);
#endif
*/

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

    // Step 4: gather points that lie on one vertical line - it is equals to merging original detected lines
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


    // Step 5: Acquire quality metrics for each pair of lines
    float bestQuality = INFINITY;
    cv::Vec4f bestLine1, bestLine2;
    bool passedThreshold = false;

    for (int i = 0; i < mergedLines.size(); i++) {
        for (int j = 0; j < mergedLines.size(); j++) {
            if (i == j)
                continue;

            cv::Vec4f line1 = mergedLines[i];
            cv::Vec4f line2 = mergedLines[j];

            // 1. How angles of polygon close to PI/2
            float angleDiff1 = std::abs(CV_PI / 2 - getAngle(line1[2], line1[3], line1[0], line1[1], line2[2], line2[3]));
            float angleDiff2 = std::abs(CV_PI / 2 - getAngle(line1[0], line1[1], line1[2], line1[3], line2[0], line2[1]));
            float angleDiff3 = std::abs(CV_PI / 2 - getAngle(line2[2], line2[3], line2[0], line2[1], line1[2], line1[3]));
            float angleDiff4 = std::abs(CV_PI / 2 - getAngle(line2[0], line2[1], line2[2], line2[3], line1[0], line1[1]));

            float angleQuality = std::max({angleDiff1, angleDiff2, angleDiff3, angleDiff4});

            if (angleQuality > 5.0f)
                continue;

            // 2. How equal vertical and horizontal sides
            float verticalRelation = getLength(line2) / getLength(line1);
            if (verticalRelation > 1.0f)
                verticalRelation = 1.0f / verticalRelation;
            float horizontalRelation = getDistance(line1[0], line1[1], line2[0], line2[1]) /
                                       getDistance(line1[2], line1[3], line2[2], line2[3]);
            if (horizontalRelation > 1.0f)
                horizontalRelation = 1.0f / horizontalRelation;

            float verticalRelationQuality = 1.0f - verticalRelation;
            float horizontalRelationQuality = 1.0f - horizontalRelation;

            // 3. How area of polygon differs from area of its bounding rectangle
            std::vector<cv::Point2f> contour, hull;
            contour.emplace_back(line1[0], line1[1]);
            contour.emplace_back(line1[2], line1[3]);
            contour.emplace_back(line2[0], line2[1]);
            contour.emplace_back(line2[2], line2[3]);
            cv::convexHull(contour, hull);
            cv::Rect boundingRect = cv::boundingRect(contour);

            float contourArea = cv::contourArea(hull);
            float rectArea = boundingRect.area();
            float squareRelation = contourArea / rectArea;
            if (squareRelation > 1.0f)
                squareRelation = 1.0f /squareRelation;

            float squareQuality = 1.0f -squareRelation;

            // 4. How equal horizontal and vertical sides
            float sideRelation1 = getLength(line1) / getLength(cv::Vec4f(line1[2], line1[3], line2[2], line2[3]));
            if (sideRelation1 > 1.0f)
                sideRelation1 = 1.0f / sideRelation1;
            float sideRelation2 = getLength(line1) / getLength(cv::Vec4f(line1[0], line1[1], line2[0], line2[1]));
            if (sideRelation2 > 1.0f)
                sideRelation2 = 1.0f / sideRelation2;
            float sideRelation3 = getLength(line2) / getLength(cv::Vec4f(line2[2], line2[3], line1[2], line1[3]));
            if (sideRelation3 > 1.0f)
                sideRelation3 = 1.0f / sideRelation3;
            float sideRelation4 = getLength(line2) / getLength(cv::Vec4f(line2[0], line2[1], line1[0], line1[1]));
            if (sideRelation4 > 1.0f)
                sideRelation4 = 1.0f / sideRelation4;

            float sidesQuality = 1.0f - std::min({sideRelation1, sideRelation2, sideRelation3, sideRelation4});

            float totalQuality = verticalRelationQuality + horizontalRelationQuality + squareQuality + sidesQuality + angleQuality;
            if (totalQuality > 1.5f)
                continue;

            passedThreshold = true;
            if (totalQuality < bestQuality) {
                bestQuality = totalQuality;
                bestLine1 = line1;
                bestLine2 = line2;
            }
        }
    }

    if (!passedThreshold)
        return GateDescriptor::noGates();


    // Step 6: Form gate from lines with best quality metrics
    cv::Vec4f leftLine, rightLine;
    if (bestLine1[2] < bestLine2[2]) {
        leftLine = bestLine1;
        rightLine = bestLine2;
    } else {
        leftLine = bestLine2;
        rightLine = bestLine1;
    }

    cv::Point2f topLeft = cv::Point2f(leftLine[2], leftLine[3]);
    cv::Point2f topRight = cv::Point2f(rightLine[2], rightLine[3]);
    cv::Point2f bottomLeft = cv::Point2f(leftLine[0], leftLine[1]);
    cv::Point2f bottomRight = cv::Point2f(rightLine[0], rightLine[1]);

    return GateDescriptor::create({topLeft, topRight, bottomRight, bottomLeft});
}


float GateDetector::getAngle(float x1, float y1, float x2, float y2, float x3, float y3) {
    float dx21 = x2-x1;
    float dx31 = x3-x1;
    float dy21 = y2-y1;
    float dy31 = y3-y1;
    float m12 = sqrt( dx21*dx21 + dy21*dy21 );
    float m13 = sqrt( dx31*dx31 + dy31*dy31 );
    return acos( (dx21*dx31 + dy21*dy31) / (m12 * m13) );
}