#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <opencv2/ximgproc.hpp>

#include "../../include/mat/MatDetector.h"
#include "../../include/mat/MatDescriptor.h"
#include "../../include/util/ImgprocPipeline.h"
#include "../../include/util/ImgprocUtil.h"
#include "../../include/util/ColorPicker.h"

cv::Mat color_picker_1;
cv::Mat color_picker_2;

std::vector<cv::Scalar> call_back(const cv::Mat& src) {
    cv::Mat hsv;

    /// Convert from BGR to HSV colorspace
    cv::cvtColor(src, hsv, CV_RGB2HSV);
    color_picker_1 = hsv;

    cv::namedWindow("Pick Color");
    if (!src.empty()) cv::imshow("Pick Color", src);

    cv::setMouseCallback("Pick Color", color_picker, 0);

    cv::Scalar m, v;
    cv::meanStdDev(color_picker_2, m, v);
    //std::cerr << "mean, var (Random Color Picker): " << std::endl;
    //std::cerr << m[0] << " " << m[1] << " " << m[2] << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;

    std::vector<cv::Scalar> limits (2);

    //limits[0] = cv::Scalar(m[0]-v[0], m[1]-v[1], m[2]-v[2]); // Mean - var for low
    //limits[1] = cv::Scalar(m[0]+v[0], m[1]+v[1], m[2]+v[2]); // Mean + var for high

    limits[0] = m;
    limits[1] = v;

    return limits;
}

/// Mouse callback function (returns color of the place where you clicked)
void color_picker(int event, int x, int y, int f, void *)
{
    if (event==1)
    {
        int r = 3;
        int off[9*2] = {0,0, -r,-r, -r,0, -r,r, 0,r, r,r, r,0, r,-r, 0,-r};
        for (int i=0; i < 9; i++)
        {
            cv::Vec3b p = color_picker_1.at<cv::Vec3b>(y+off[2*i], x+off[2*i+1]);
            std::cerr << int(p[0]) << " " << int(p[1]) << " " << int(p[2]) << std::endl;
            color_picker_2.push_back(p);
        }
    }
}

/// Usage:
/**
    cv::Mat hsv;

    /// Convert from BGR to HSV colorspace
    cv::cvtColor(src, hsv, CV_RGB2HSV);

    std::vector<cv::Scalar> limits = call_back (hsv);

    cv::Scalar m, v;
    m = limits[0];
    v = limits[1];
 */


