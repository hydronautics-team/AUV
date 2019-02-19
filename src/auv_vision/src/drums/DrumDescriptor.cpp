#include "../../include/drums/DrumDescriptor.h"
#include "../../include/util/ImgprocUtil.h"

DrumDescriptor DrumDescriptor::noDrums() {return DrumDescriptor(false, false, std::vector<cv::Vec3f>(), std::vector<cv::Vec3f>());}

DrumDescriptor DrumDescriptor::create(const std::vector<cv::Vec3f>& redDrums, const std::vector<cv::Vec3f>& blueDrums) {
    if ((!redDrums.empty()) && (!blueDrums.empty())) return DrumDescriptor(true, true, redDrums, blueDrums);
    else if (!redDrums.empty()) return DrumDescriptor(true, false, redDrums, std::vector<cv::Vec3f>());
    else return DrumDescriptor(false, true,  std::vector<cv::Vec3f>(), blueDrums);
}

DrumDescriptor::DrumDescriptor(bool redDrum, bool blueDrum, const std::vector<cv::Vec3f>& redDrums, const std::vector<cv::Vec3f>& blueDrums) {
    this->redDrum = redDrum;
    this->blueDrum = blueDrum;
    if (redDrum)
        this->redDrums = redDrums;
    if (blueDrum)
        this->blueDrums = blueDrums;
}

DrumDescriptor::DrumDescriptor(const DrumDescriptor &other) {
    this->redDrum = other.redDrum;
    this->blueDrum = other.blueDrum;
    this->redDrums = other.redDrums;
    this->blueDrums = other.blueDrums;
}

DrumDescriptor& DrumDescriptor::operator=(const DrumDescriptor &other) {
    if (this != &other) {
        this->redDrum = other.redDrum;
        this->blueDrum = other.blueDrum;
        this->redDrums = other.redDrums;
        this->blueDrums = other.blueDrums;
    }
    return *this;
}

bool DrumDescriptor::hasDrumCenter() {return redDrum || blueDrum;}
bool DrumDescriptor::hasRedDrumCenter() {return redDrum;}
bool DrumDescriptor::hasBlueDrumCenter() {return blueDrum;}

cv::Point2f DrumDescriptor::getRedDrumCenter(const int i) {return cv::Point2f(redDrums[i][0], redDrums[i][1]);}
cv::Point2f DrumDescriptor::getBlueDrumCenter(const int i) {return cv::Point2f(blueDrums[i][0], blueDrums[i][1]);}

std::vector<cv::Vec3f> DrumDescriptor::getAllRedDrums() {
    return redDrums;

}
std::vector<cv::Vec3f> DrumDescriptor::getAllBlueDrums() {
    return blueDrums;
}

