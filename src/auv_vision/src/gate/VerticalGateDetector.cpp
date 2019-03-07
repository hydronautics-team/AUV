#include "gate/VerticalGateDetector.h"

VerticalGateDetector::VerticalGateDetector(float horizontalToVerticalRelation) : IGateDetector(horizontalToVerticalRelation) {}

GateDescriptor VerticalGateDetector::findBestByQuality(const std::vector<cv::Vec4f> &verticalLines, long frameWidth,
                                                       long frameHeight) {

    std::vector<std::pair<cv::Vec4f, cv::Vec4f>> candidates;

    for (int i = 0; i < verticalLines.size(); i++) {
        for (int j = 0; j < verticalLines.size(); j++) {
            if (i == j)
                continue;
            cv::Vec4f vertical1, vertical2; // 1 - left, 2 - right
            if (verticalLines[i][0] < verticalLines[j][0]) {
                vertical1 = verticalLines[i];
                vertical2 = verticalLines[j];
            } else {
                vertical1 = verticalLines[j];
                vertical2 = verticalLines[i];
            }

            float parallelRelation = getLength(vertical1) / getLength(vertical2);
            if (parallelRelation > 1.0f)
                parallelRelation = getLength(vertical2) / getLength(vertical1);
            if (parallelRelation < parallelRelationThreshold)
                continue;

            cv::Vec4f horizontalTop = cv::Vec4f(vertical1[0], vertical1[1], vertical2[0], vertical2[1]);
            cv::Vec4f horizontalBottom = cv::Vec4f(vertical1[2], vertical1[3], vertical2[2], vertical2[3]);
            float horizontalSlope = std::max(getLineSlopeHorizontal(horizontalTop), getLineSlopeHorizontal(horizontalBottom));
            if (horizontalSlope > horizontalSlopeThreshold)
                continue;

            float relationError1 = std::abs(horizontalToVerticalRelation - getLength(horizontalTop) / getLength(vertical1));
            float relationError2 = std::abs(horizontalToVerticalRelation - getLength(horizontalTop) / getLength(vertical2));
            float relationError3 = std::abs(horizontalToVerticalRelation - getLength(horizontalBottom) / getLength(vertical1));
            float relationError4 = std::abs(horizontalToVerticalRelation - getLength(horizontalBottom) / getLength(vertical2));
            float relationError = std::min({relationError1, relationError2, relationError3, relationError4});
            if (relationError > orthogonalRelationErrorThreshold)
                continue;

            float area = std::max(getLength(vertical1), getLength(vertical2)) *
                         std::max(getLength(horizontalTop), getLength(horizontalBottom));
            if (area / (float)(frameHeight * frameWidth) < areaFrameRelationThreshold)
                continue;

            candidates.emplace_back(vertical1, vertical2);
        }
    }

    if (candidates.empty())
        return GateDescriptor::noGates();

    float maxArea = 0.0f;
    std::pair<cv::Vec4f, cv::Vec4f> winner;
    for (int i = 0; i < candidates.size(); i++) {
        cv::Vec4f horizontal = cv::Vec4f(candidates[i].first[0], candidates[i].first[1],
                                         candidates[i].second[0], candidates[i].second[1]);
        cv::Vec4f vertical = std::max(getLength(candidates[i].first), getLength(candidates[i].second));
        float area = getLength(horizontal) * getLength(vertical);
        if (area > maxArea) {
            maxArea = area;
            winner = candidates[i];
        }
    }

    cv::Point2f topLeft = cv::Point2f(winner.first[0], winner.first[1]);
    cv::Point2f topRight = cv::Point2f(winner.second[0], winner.second[1]);
    cv::Point2f bottomLeft = cv::Point2f(winner.first[2], winner.first[3]);
    cv::Point2f bottomRight = cv::Point2f(winner.second[2], winner.first[3]);

    return GateDescriptor::create({topLeft, topRight, bottomRight, bottomLeft});
}

GateDescriptor VerticalGateDetector::detect(const cv::Mat &src) {
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

    return findBestByQuality(mergedVertical, src.cols, src.rows);
}

void VerticalGateDetector::setParallelRelationThreshold(float parallelRelationThreshold) {
    this->parallelRelationThreshold = parallelRelationThreshold;
}

void VerticalGateDetector::setHorizontalSlopeThreshold(float horizontalSlopeThreshold) {
    this->horizontalSlopeThreshold = horizontalSlopeThreshold;
}

void VerticalGateDetector::setOrthogonalRelationErrorThreshold(float orthogonalRelationErrorThreshold) {
    this->orthogonalRelationErrorThreshold = orthogonalRelationErrorThreshold;
}

void VerticalGateDetector::setAreaFrameRelationThreshold(float areaFrameRelationThreshold) {
    this->areaFrameRelationThreshold = areaFrameRelationThreshold;
}

void VerticalGateDetector::reconfigure(auv_vision::GateLocatorConfig &config, uint32_t level) {
    IGateDetector::reconfigure(config, level);
    setParallelRelationThreshold(config.vertical_parallelRelationThreshold);
    setHorizontalSlopeThreshold(config.vertical_horizontalSlopeThreshold);
    setOrthogonalRelationErrorThreshold(config.vertical_orthogonalRelationErrorThreshold);
    setAreaFrameRelationThreshold(config.vertical_areaFrameRelationThreshold);
}


