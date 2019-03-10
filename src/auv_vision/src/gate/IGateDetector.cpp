#include "gate/IGateDetector.h"

IGateDetector::IGateDetector(float horizontalToVerticalRelation) :
    horizontalToVerticalRelation(horizontalToVerticalRelation) {}

void IGateDetector::setPublisher(const ros::NodeHandle& nh) {
    image_transport::ImageTransport it(nh);
    imagePublisher = it.advertise("/gate/lines", 100);
}

void IGateDetector::detectLines(const cv::Mat &image, std::vector<cv::Vec4f> &lines) {

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

float IGateDetector::getLineSlopeVertical(const cv::Vec4f &line) {
    if (line[1] == line[3])
        return 90.0f;
    if (line[0] == line[2])
        return 0.0f;
    float tan = std::abs(line[0] - line[2]) / std::abs(line[1] - line[3]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

float IGateDetector::getLineSlopeHorizontal(const cv::Vec4f &line) {
    if (line[0] == line[2])
        return 0.0f;
    if (line[1] == line[3])
        return 90.0f;
    float tan = std::abs(line[1] - line[3]) / std::abs(line[0] - line[2]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

void IGateDetector::filterVerticalAndHorizontal(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &vertical,
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

float IGateDetector::getLength(const cv::Vec4f &line) {
    return std::sqrt((line[2] - line[0]) * (line[2] - line[0]) + (line[3] - line[1]) * (line[3] - line[1]));
}

void IGateDetector::mergeVerticalLines(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines) {

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

bool IGateDetector::isAbleToVerticalMerge(const cv::Vec4f &line1, const cv::Vec4f &line2) {


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

void IGateDetector::mergeY(const std::vector<cv::Vec4f> &lines, std::vector<cv::Vec4f> &mergedLines) {

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

cv::Vec4f IGateDetector::createVerticalLine(const std::vector<cv::Vec4f> &lines) {
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

void IGateDetector::setVerticalSlope(float verticalSlope) {
    this->verticalSlope = verticalSlope;
}

void IGateDetector::setHorizontalSlope(float horizontalSlope) {
    this->horizontalSlope = horizontalSlope;
}

void IGateDetector::setLengthRelation(float lengthRelation) {
    this->lengthRelation = lengthRelation;
}

void IGateDetector::setMergingLineDistanceHorizontal(float mergingLineDistanceHorizontal) {
    this->mergingLineDistanceHorizontal = mergingLineDistanceHorizontal;
}

void IGateDetector::setMergingLineDistanceVertical(float mergingLineDistanceVertical) {
    this->mergingLineDistanceVertical = mergingLineDistanceVertical;
}

void IGateDetector::setLength_threshold(int length_threshold) {
    this->length_threshold = length_threshold;
    updateFLD();
}

void IGateDetector::setDistance_threshold(float distance_threshold) {
    this->distance_threshold = distance_threshold;
    updateFLD();
}

void IGateDetector::setCanny_th1(double canny_th1) {
    this->canny_th1 = canny_th1;
    updateFLD();
}

void IGateDetector::setCanny_th2(double canny_th2) {
    this->canny_th2 = canny_th2;
    updateFLD();
}

void IGateDetector::setCanny_aperture_size(int canny_aperture_size) {
    this->canny_aperture_size = canny_aperture_size;
    updateFLD();
}

void IGateDetector::updateFLD() {
    fld =
            cv::ximgproc::createFastLineDetector(length_threshold,
                                                 distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);
}

void IGateDetector::reconfigure(auv_vision::GateLocatorConfig &config, uint32_t level) {
    setVerticalSlope(config.verticalSlope);
    setHorizontalSlope(config.horizontalSlope);
    setLengthRelation(config.lengthRelation);
    setMergingLineDistanceHorizontal(config.mergingLineDistanceHorizontal);
    setMergingLineDistanceVertical(config.mergingLineDistanceVertical);
    setLength_threshold(config.fld_length_threshold);
    setDistance_threshold(config.fld_distance_threshold);
    setCanny_th1(config.fld_canny_threshold_1);
    setCanny_th2(config.fld_canny_threshold_2);
    if (config.fld_canny_aperture == 3.0 || config.fld_canny_aperture == 5.0 || config.fld_canny_aperture == 7.0)
        setCanny_aperture_size(config.fld_canny_aperture);
}
