#include "../../include/util/ImgprocPipeline.h"

ImgprocPipe::ImgprocPipe(const cv::Mat& currentImage, bool debug) {
    this->currentImage = currentImage;
    this->debug = debug;
}

ImgprocPipe::ImgprocPipe(ImgprocPipe &other) {
    this->currentImage = other.currentImage;
    this->debug = other.debug;
}

ImgprocPipe& ImgprocPipe::operator=(const ImgprocPipe &other) {
    if (this != &other) {
        this->currentImage = other.currentImage;
        this->debug = other.debug;
    }
    return *this;
}

ImgprocPipe ImgprocPipe::apply(std::function<cv::Mat(const cv::Mat &)> imgprocFuntion, const char* name) {
    cv::Mat newImage = imgprocFuntion(currentImage);
    if (debug) {
        cv::namedWindow(name);
        cv::imshow(name, newImage);
        cv::waitKey();
        cv::destroyWindow(name);
    }
    ImgprocPipe pipe(newImage, debug);
    return pipe;
}

ImgprocPipe ImgprocPipe::apply(std::function<void(const cv::Mat &, cv::Mat &)> imgprocFuntion, const char* name) {
    cv::Mat newImage;
    imgprocFuntion(currentImage, newImage);
    if (debug) {
        cv::namedWindow(name);
        cv::imshow(name, newImage);
        cv::waitKey();
        cv::destroyWindow(name);
    }
    ImgprocPipe pipe(newImage, debug);
    return pipe;
}

cv::Mat ImgprocPipe::getImage() {
    return currentImage;
}