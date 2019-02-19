#ifndef AUV_VISION_COLORPICKER_H
#define AUV_VISION_COLORPICKER_H

void color_picker(int event, int x, int y, int f, void *);
std::vector<cv::Scalar> call_back(const cv::Mat& src);

#endif //AUV_VISION_COLORPICKER_H
