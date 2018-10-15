#ifndef AUV_VISION_IMGPROCPIPELINE_H
#define AUV_VISION_IMGPROCPIPELINE_H

#include <functional>
#include <opencv2/opencv.hpp>

class ImgprocPipe {

private:

    cv::Mat currentImage;
    bool debug;

public:

    ImgprocPipe(const cv::Mat& currentImage, bool debug);
    ImgprocPipe(ImgprocPipe& other);
    ~ImgprocPipe() = default;

    ImgprocPipe& operator=(const ImgprocPipe& other);

    ImgprocPipe apply(std::function<cv::Mat(const cv::Mat&)> imgprocFuntion, const char* name = "Operation");
    ImgprocPipe apply(std::function<void(const cv::Mat&, cv::Mat&)> imgprocFuntion, const char* name = "Operation");
    cv::Mat getImage();

};

static ImgprocPipe createPipeline(const cv::Mat src, bool debug) {
    ImgprocPipe pipe(src, debug);
    return pipe;
}

#endif //AUV_VISION_IMGPROCPIPELINE_H
