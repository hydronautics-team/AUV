#ifndef AUV_VISION_DRUMDETECTOR_H
#define AUV_VISION_DRUMDETECTOR_H

#include <opencv2/opencv.hpp>
#include "DrumDescriptor.h"

class DrumDetector {

private:

    float lower_blue_H = 100;
    float higher_blue_H = 180;
    float lower_blue_S = 35;
    float higher_blue_S = 255;
    float lower_blue_V = 50;
    float higher_blue_V = 255;

    float lower_red_1_H = 0;
    float higher_red_1_H = 70;
    float lower_red_2_H = 160;
    float higher_red_2_H = 180;

    float lower_red_1_S = 0;
    float higher_red_1_S = 255;
    float lower_red_2_S = 62;
    float higher_red_2_S = 255;

    float lower_red_1_V = 50;
    float higher_red_1_V = 170;
    float lower_red_2_V = 60;
    float higher_red_2_V = 255;

    float min_dist = 400;
    float param_1 = 100;
    float param_2 = 60;


    void defaultPreprocess(const cv::Mat& src, cv::Mat& dst); // Image filter
    void extractValueChannel(const cv::Mat& src, cv::Mat& dst);
    void detectContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours);
    std::vector<cv::Vec3f> findCircles(const cv::Mat& src, cv::Mat& dst);

    void morphology(const cv::Mat& src, cv::Mat& dst);
    void meanShift(const cv::Mat& src, cv::Mat& dst);

public:

    cv::Mat getreconfImageAfterMorphology();
    cv::Mat getreconfImageAfterMask();
    cv::Mat getreconfImageAfterMaskRED();
    cv::Mat getreconfImageAfterMaskBLUE();
    cv::Mat getreconfImageAfterColorEnhancement();
    cv::Mat getreconfmaskedImage_red();
    cv::Mat getreconfmaskedImage_blue();


    /// BLUE H channel
    float setLowerBlueH() const;
    void setLowerBlueH(float lower_blue_H);


    float setHigherBlueH() const;
    void setHigherBlueH(float higher_blue_H);

    /// BLUE S channel
    float setLowerBlueS() const;
    void setLowerBlueS(float lower_blue_S);


    float setHigherBlueS() const;
    void setHigherBlueS(float higher_blue_S);

    /// BLUE V channel
    float setLowerBlueV() const;
    void setLowerBlueV(float lower_blue_V);


    float setHigherBlueV() const;
    void setHigherBlueV(float higher_blue_V);

/******************************************************/

    /// RED H channel
    float setLowerRED_1H() const;
    void setLowerRED_1H(float lower_red_1_H);
    float setHigherRED_1H() const;
    void setHigherRED_1H(float higher_red_1_H);


    float setLowerRED_2H() const;
    void setLowerRED_2H(float lower_red_2_H);
    float setHigherRED_2H() const;
    void setHigherRED_2H(float higher_red_2_H);

    /// RED S channel
    float setLowerRED_1S() const;
    void setLowerRED_1S(float lower_red_1_S);
    float setHigherRED_1S() const;
    void setHigherRED_1S(float higher_red_1_S);

    float setLowerRED_2S() const;
    void setLowerRED_2S(float lower_red_2_S);
    float setHigherRED_2S() const;
    void setHigherRED_2S(float higher_red_2_S);

    /// RED V channel
    float setLowerRED_1V() const;
    void setLowerRED_1V(float lower_red_1_V);
    float setHigherRED_1V() const;
    void setHigherRED_1V(float higher_red_1_V);

    float setLowerRED_2V() const;
    void setLowerRED_2V(float lower_red_2_V);
    float setHigherRED_2V() const;
    void setHigherRED_2V(float higher_red_2_V);


    float setMinDist() const;
    void setMinDist(float min_dist);

    float setParam1() const;
    void setParam1(float param_1);

    float setParam2() const;
    void setParam2(float param_2);

    bool isRed(const cv::Mat& src, const cv::Vec3f circle);
    bool isBlue(const cv::Mat& src, const cv::Vec3f circle);

    DrumDetector() = default;
    ~DrumDetector() = default;
    DrumDetector& operator=(const DrumDetector& other) = default;

    DrumDescriptor detect(const cv::Mat& src, bool withPreprocess);

};

#endif //AUV_VISION_DRUMDETECTOR_H
