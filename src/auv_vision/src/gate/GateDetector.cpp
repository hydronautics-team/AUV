#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "../../include/gate/GateDetector.h"
#include "../../include/gate/GateDescriptor.h"
#include "../../include/util/ImgprocPipeline.h"


void GateDetector::setPublisher(const ros::NodeHandle& nh) {
    image_transport::ImageTransport it(nh);
    imagePublisher = it.advertise("/gate/lines", 100);
}

void GateDetector::detectLines(const cv::Mat &image, std::vector<cv::Vec4f> &lines) {

    std::vector<cv::Vec4f> detectedLines;
    fld->detect(image, detectedLines);

    cv::Mat canvas;
    image.copyTo(canvas);
    fld->drawSegments(canvas, detectedLines);
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", canvas).toImageMsg();
    imagePublisher.publish(imageMsg);

    // Sort verticies in lines: top vertix is first
    for (int i = 0; i < detectedLines.size(); i++) {
        if (detectedLines[i][1] < detectedLines[i][3])
            lines.push_back(detectedLines[i]);
        else
            lines.emplace_back(detectedLines[i][2], detectedLines[i][3], detectedLines[i][0], detectedLines[i][1]);
    }
}

float GateDetector::getLineSlopeVertical(const cv::Vec4f &line) {
    if (line[1] == line[3])
        return 90.0f;
    if (line[0] == line[2])
        return 0.0f;
    float tan = std::abs(line[0] - line[2]) / std::abs(line[1] - line[3]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

float GateDetector::getLineSlopeHorizontal(const cv::Vec4f &line) {
    if (line[0] == line[2])
        return 0.0f;
    if (line[1] == line[3])
        return 90.0f;
    float tan = std::abs(line[1] - line[3]) / std::abs(line[0] - line[2]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

void GateDetector::filterVerticalAndHorizontal(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &vertical,
                                           std::vector<cv::Vec4f> &horizontal) {

    std::vector<cv::Vec4f> filteredVerticalLines;
    std::vector<cv::Vec4f> filteredHorizontalLines;
    double maxLengthVertical = 0;
    double maxLengthHorizontal = 0;


    for (int i = 0; i < lines.size(); i++) {
        if (getLineSlopeVertical(lines[i]) < verticalSlope) {
            filteredVerticalLines.push_back(lines[i]);
            double length = getLength(lines[i]);
            if (length > maxLengthVertical)
                maxLengthVertical = length;
        }

        else if (getLineSlopeHorizontal(lines[i]) < horizontalSlope) {
            filteredHorizontalLines.push_back(lines[i]);
            double length = getLength(lines[i]);
            if (length > maxLengthHorizontal)
                maxLengthHorizontal = length;
        }
    }

    for (int i = 0; i < filteredVerticalLines.size(); i++) {
        double length = getLength(filteredVerticalLines[i]);
        if (length >= lengthRelation * maxLengthVertical)
            vertical.push_back(filteredVerticalLines[i]);
    }

    for (int i = 0; i < filteredHorizontalLines.size(); i++) {
        double length = getLength(filteredHorizontalLines[i]);
        if (length >= lengthRelation * maxLengthVertical)
            horizontal.push_back(filteredHorizontalLines[i]);
    }
}

float GateDetector::getLength(const cv::Vec4f &line) {
    return std::sqrt((line[2] - line[0]) * (line[2] - line[0]) + (line[3] - line[1]) * (line[3] - line[1]));
}

GateDescriptor GateDetector::detect(const cv::Mat &src) {

    // Step 1: Convert BGR image to Grayscale
    cv::Mat image;
    cv::cvtColor(src, image, CV_BGR2GRAY);

    // Step 2: Find all lines
    std::vector<cv::Vec4f> allLines;
    detectLines(image, allLines);
    if (allLines.size() < 3)
        return GateDescriptor::noGates();

    // Step 3: Filter vertical lines
    std::vector<cv::Vec4f> verticalLines, horizontalLines;
    filterVerticalAndHorizontal(allLines, verticalLines, horizontalLines);

    // Step 4: Merge vertical lines
    std::vector<cv::Vec4f> mergedVertical;
    mergeVerticalLines(verticalLines, mergedVertical);


    return findBestByQuality(mergedVertical, horizontalLines, src.rows * src.cols);
}


void GateDetector::mergeVerticalLines(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines) {

    std::vector<cv::Vec4f> allLines = lines;
    std::sort(allLines.begin(), allLines.end(),
              [](const cv::Vec4f& a, const cv::Vec4f& b) -> bool {
                  if (a[2] != b[2])
                      return a[2] > b[2];
                  return a[1] < b[1];
              });

    cv::Vec4f currentLine = allLines[0];
    std::vector<cv::Vec4f> group;
    group.push_back(currentLine);
    for (int i = 1; i < allLines.size(); i++) {
        cv::Vec4f nextLine = allLines[i];
        if (std::min(std::abs(currentLine[0] - nextLine[0]), std::abs(currentLine[2] - nextLine[2])) <= mergingLineDistanceHorizontal) {
            group.push_back(nextLine);
        } else {
            std::vector<cv::Vec4f> linesToMerge;
            mergeY(group, linesToMerge);
            mergedLines.insert(mergedLines.end(), linesToMerge.begin(), linesToMerge.end());
            group.clear();
            group.push_back(nextLine);
        }

        currentLine = nextLine;

    }
    if (!group.empty()) {
        std::vector<cv::Vec4f> linesToMerge;
        mergeY(group, linesToMerge);
        mergedLines.insert(mergedLines.end(), linesToMerge.begin(), linesToMerge.end());
    }
}

bool GateDetector::isAbleToVerticalMerge(const cv::Vec4f &line1, const cv::Vec4f &line2) {


    cv::Vec4f high, low;
    if (line1[1] < line2[1]) {
        high = line1;
        low = line2;
    } else {
        high = line2;
        low = line1;
    }

    if (low[1] < high[3])
        return true;

    return low[1] - high[3] < mergingLineDistanceVertical;
}

void GateDetector::mergeY(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines) {

    std::vector<cv::Vec4f> allLines = lines;
    std::sort(allLines.begin(), allLines.end(),
              [](const cv::Vec4f& a, const cv::Vec4f& b) -> bool {
                  return a[1] > b[1];
              });

    cv::Vec4f currentLine = allLines[0];
    std::vector<cv::Vec4f> currentGroup;
    currentGroup.push_back(currentLine);
    for (int j = 1; j < allLines.size(); j++) {
        cv::Vec4f next = allLines[j];
        if (isAbleToVerticalMerge(currentLine, next)) {
            currentGroup.push_back(next);
        } else {
            mergedLines.push_back(createVerticalLine(currentGroup));
            currentGroup.clear();
            currentGroup.push_back(next);
        }
        currentLine= next;
    }
    if (!currentGroup.empty()) {
        mergedLines.push_back(createVerticalLine(currentGroup));
    }
}

cv::Vec4f GateDetector::createVerticalLine(const std::vector<cv::Vec4f> &lines) {
    float minY = INFINITY;
    float maxY = 0.0f;
    float minX = INFINITY;
    float maxX = 0.0f;

    for (int i = 0; i < lines.size(); i++) {
        if (lines[i][0] < minX)
            minX = lines[i][0];
        if (lines[i][0] > maxX)
            maxX = lines[i][0];

        if (lines[i][1] < minY)
            minY = lines[i][1];
        if (lines[i][3] > maxY)
            maxY = lines[i][3];

        if (lines[i][2] < minX)
            minX = lines[i][2];
        if (lines[i][2] > maxX)
            maxX = lines[i][2];
    }

    float x = (minX + maxX) / 2.0f;

    return cv::Vec4f(x, minY, x, maxY);
}


GateDescriptor GateDetector::findBestByQuality(const std::vector<cv::Vec4f> &verticalLines,
                                           const std::vector<cv::Vec4f> &horizontalLines,
                                           long frameArea) {

    std::vector<std::pair<cv::Vec4f, cv::Vec4f>> candidates;

    for (int i = 0; i < verticalLines.size(); i++) {
        cv::Vec4f vertical = verticalLines[i];
        for (int j = 0; j < horizontalLines.size(); j++) {
            cv::Vec4f horizontal = horizontalLines[j];
            if (horizontal[0] > horizontal[2])
                horizontal = cv::Vec4f(horizontal[2], horizontal[3], horizontal[0], horizontal[1]);


            float overlap = std::max(horizontal[1], horizontal[3]) - vertical[1];
            if ((overlap > 0.0f) && (overlap > overlapThreshold))
                continue;

            float distX = std::min(std::abs(vertical[0] - horizontal[0]),
                                   std::abs(vertical[0] - horizontal[2]));
            if (distX > distXThreshold)
                continue;

            float distY = std::min(std::abs(vertical[1] - horizontal[1]),
                                   std::abs(vertical[1] - horizontal[3]));
            if (distY > distYThreshold)
                continue;

            float sideRelation = 0.0f;
            if (getLength(vertical) > getLength(horizontal))
                sideRelation = getLength(horizontal) / getLength(vertical);
            else
                sideRelation = getLength(vertical) / getLength(horizontal);
            if (sideRelation < sidesRelationThreshold)
                continue;

            float coordVertX = vertical[0] - vertical[2];
            float coordVertY = vertical[3] - vertical[1];
            float coordHorX = horizontal[2] - horizontal[0];
            float coordHorY = horizontal[3] - horizontal[1];
            float cosine = (coordVertX * coordHorX + coordVertY * coordHorY) / (getLength(horizontal) * getLength(vertical));
            float angleDiff = std::abs(90.0f - (std::acos(cosine) * 180.0f / CV_PI));
            if (angleDiff > angleDiffThreshold)
                continue;

            float area = getLength(vertical) * getLength(horizontal);
            if (area / (float)frameArea < areaFrameRelationThreshold)
                continue;

            candidates.emplace_back(vertical, horizontal);
        }
    }

    if (candidates.empty())
        return GateDescriptor::noGates();

    float maxArea = 0.0f;
    std::pair<cv::Vec4f, cv::Vec4f> winner;
    for (int i = 0; i < candidates.size(); i++) {
        float area = getLength(candidates[i].first) * getLength(candidates[i].second);
        if (area > maxArea) {
            maxArea = area;
            winner = candidates[i];
        }
    }

    cv::Point2f topLeft = cv::Point2f(winner.second[0], winner.second[1]);
    cv::Point2f topRight = cv::Point2f(winner.second[2], winner.second[3]);
    cv::Point2f bottomLeft = cv::Point2f(winner.second[0], winner.first[3]);
    cv::Point2f bottomRight = cv::Point2f(winner.second[2], winner.first[3]);

    return GateDescriptor::create({topLeft, topRight, bottomRight, bottomLeft});
}


void GateDetector::setVerticalSlope(float verticalSlope) {
    this->verticalSlope = verticalSlope;
}

void GateDetector::setHorizontalSlope(float horizontalSlope) {
    this->horizontalSlope = horizontalSlope;
}

void GateDetector::setLengthRelation(float lengthRelation) {
    this->lengthRelation = lengthRelation;
}

void GateDetector::setMergingLineDistanceHorizontal(float mergingLineDistanceHorizontal) {
    this->mergingLineDistanceHorizontal = mergingLineDistanceHorizontal;
}

void GateDetector::setMergingLineDistanceVertical(float mergingLineDistanceVertical) {
    this->mergingLineDistanceVertical = mergingLineDistanceVertical;
}

void GateDetector::setOverlapThreshold(float overlapThreshold) {
    this->overlapThreshold = overlapThreshold;
}

void GateDetector::setDistXThreshold(float distXThreshold) {
    this->distXThreshold = distXThreshold;
}

void GateDetector::setDistYThreshold(float distYThreshold) {
    this->distYThreshold = distYThreshold;
}

void GateDetector::setSidesRelationThreshold(float sidesRelationThreshold) {
    this->sidesRelationThreshold = sidesRelationThreshold;
}

void GateDetector::setAngleDiffThreshold(float angleDiffThreshold) {
    this->angleDiffThreshold = angleDiffThreshold;
}

void GateDetector::setAreaFrameRelationThreshold(float areaFrameRelationThreshold) {
    this->areaFrameRelationThreshold = areaFrameRelationThreshold;
}

void GateDetector::setLength_threshold(int length_threshold) {
    this->length_threshold = length_threshold;
    updateFLD();
}

void GateDetector::setDistance_threshold(float distance_threshold) {
    this->distance_threshold = distance_threshold;
    updateFLD();
}

void GateDetector::setCanny_th1(double canny_th1) {
    this->canny_th1 = canny_th1;
    updateFLD();
}

void GateDetector::setCanny_th2(double canny_th2) {
    this->canny_th2 = canny_th2;
    updateFLD();
}

void GateDetector::setCanny_aperture_size(int canny_aperture_size) {
     this->canny_aperture_size = canny_aperture_size;
    updateFLD();
}

void GateDetector::updateFLD() {
    fld =
            cv::ximgproc::createFastLineDetector(length_threshold,
                                                 distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);
}
