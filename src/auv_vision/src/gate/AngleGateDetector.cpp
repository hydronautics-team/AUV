#include "gate/AngleGateDetector.h"

AngleGateDetector::AngleGateDetector(float horizontalToVerticalRelation) : IGateDetector(horizontalToVerticalRelation) {}

GateDescriptor AngleGateDetector::findBestByQuality(const std::vector<cv::Vec4f> &verticalLines,
                                               const std::vector<cv::Vec4f> &horizontalLines,
                                               long frameWidth, long frameHeight) {

    std::vector<std::pair<cv::Vec4f, cv::Vec4f>> candidates;

    for (int i = 0; i < verticalLines.size(); i++) {
        cv::Vec4f vertical = verticalLines[i];
        for (int j = 0; j < horizontalLines.size(); j++) {
            cv::Vec4f horizontal = horizontalLines[j];
            if (horizontal[0] > horizontal[2])
                horizontal = cv::Vec4f(horizontal[2], horizontal[3], horizontal[0], horizontal[1]);

/*            if (std::max(horizontal[1], horizontal[3]) > (1.0f - horizontalPositionRatioThreshold) * frameHeight)
                continue;*/

            if (vertical[1] > horizontal[1] && vertical[1] < horizontal[3]) {
                float overlap = std::min(vertical[1] - horizontal[1], horizontal[3] - vertical[1]);
                if ((overlap > 0.0f) && (overlap > overlapThreshold))
                    continue;
            }

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
            if (std::abs(horizontalToVerticalRelation - sideRelation) > sidesRelationErrorThreshold)
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
            if (area / (float)(frameHeight * frameWidth) < areaFrameRelationThreshold)
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

GateDescriptor AngleGateDetector::detect(const cv::Mat &src) {

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


    return findBestByQuality(mergedVertical, horizontalLines, src.cols, src.rows);
}

void AngleGateDetector::setOverlapThreshold(float overlapThreshold) {
    this->overlapThreshold = overlapThreshold;
}

void AngleGateDetector::setDistXThreshold(float distXThreshold) {
    this->distXThreshold = distXThreshold;
}

void AngleGateDetector::setDistYThreshold(float distYThreshold) {
    this->distYThreshold = distYThreshold;
}

void AngleGateDetector::setSidesRelationErrorThreshold(float sidesRelationErrorThreshold) {
    this->sidesRelationErrorThreshold = sidesRelationErrorThreshold;
}

void AngleGateDetector::setAngleDiffThreshold(float angleDiffThreshold) {
    this->angleDiffThreshold = angleDiffThreshold;
}

void AngleGateDetector::setAreaFrameRelationThreshold(float areaFrameRelationThreshold) {
    this->areaFrameRelationThreshold = areaFrameRelationThreshold;
}

void AngleGateDetector::setHorizontalPositionRatioThreshold(float horizontalPositionRatioThreshold) {
    this->horizontalPositionRatioThreshold = horizontalPositionRatioThreshold;
}

void AngleGateDetector::reconfigure(auv_vision::GateLocatorConfig &config, uint32_t level) {
    IGateDetector::reconfigure(config, level);
    setOverlapThreshold(config.angle_overlapThreshold);
    setDistXThreshold(config.angle_distXThreshold);
    setDistYThreshold(config.angle_distYThreshold);
    setSidesRelationErrorThreshold(config.angle_horizontalToVerticalRelationErroThreshold);
    setAngleDiffThreshold(config.angle_angleDiffThreshold);
    setAreaFrameRelationThreshold(config.angle_areaFrameRelationThreshold);
    setHorizontalPositionRatioThreshold(config.angle_horizontalPositionRatioThreshold);
}

