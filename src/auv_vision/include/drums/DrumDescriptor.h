#ifndef AUV_VISION_DRUMDESCRIPTOR_H
#define AUV_VISION_DRUMDESCRIPTOR_H

#include <opencv2/opencv.hpp>

class DrumDescriptor {

private:

    bool redDrum, blueDrum;

    std::vector<cv::Vec3f> redDrums;
    std::vector<cv::Vec3f> blueDrums;

    DrumDescriptor(bool redDrum, bool blueDrum, const std::vector<cv::Vec3f>& redDrums, const std::vector<cv::Vec3f>& blueDrums); /// Constructor

public:

    static DrumDescriptor noDrums();
    static DrumDescriptor create(const std::vector<cv::Vec3f>& redDrums, const std::vector<cv::Vec3f>& blueDrums);  /// Functions to initialise class

    DrumDescriptor(const DrumDescriptor& other);
    ~DrumDescriptor() = default;
    DrumDescriptor& operator=(const DrumDescriptor& other);

    bool hasDrumCenter();
    bool hasBlueDrumCenter();
    bool hasRedDrumCenter();

    std::vector<cv::Vec3f> getAllRedDrums();
    std::vector<cv::Vec3f> getAllBlueDrums();

    cv::Point2f getBlueDrumCenter(const int i);
    cv::Point2f getRedDrumCenter(const int i);

};


#endif //AUV_VISION_DRUMDESCRIPTOR_H
