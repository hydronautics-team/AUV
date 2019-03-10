#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <opencv2/ximgproc.hpp>


/**
#if CUDA_ENABLED == 0
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
*/

#include "../../include/mat/MatDetector.h"
#include "../../include/mat/MatDescriptor.h"
#include "../../include/util/ImgprocPipeline.h"
#include "../../include/util/ImgprocUtil.h"


void MatDetector::defaultPreprocess(const cv::Mat &src, cv::Mat &dst) {
    // TODO Define which pre-processing methods should be in common scope
    dst = createPipeline(src, false)
            //.apply(std::bind(&MatDetector::meanShift, this, std::placeholders::_1, std::placeholders::_2), "MeanShift")
            //.apply(std::bind(&MatDetector::extractValueChannel, this, std::placeholders::_1, std::placeholders::_2), "Value channel")
            .apply(std::bind(&MatDetector::extractGreenColour, this, std::placeholders::_1, std::placeholders::_2), "After colour filter")
            .apply(std::bind(&MatDetector::morphology, this, std::placeholders::_1, std::placeholders::_2), "Rid of noise")
            .getImage();
}

void MatDetector::meanShift(const cv::Mat &src, cv::Mat &dst) {

    // TODO Fix problems with CUDA dependencies
    cv::pyrMeanShiftFiltering(src, dst, 30, 5);
    /**
    #if CUDA_ENABLED == 0
        cv::pyrMeanShiftFiltering(src, dst, 30, 5);
    #else
        cv::cuda::GpuMat gpuSrc, gpuDest;
        gpuSrc.upload(src);
        cv::cuda::cvtColor(gpuSrc, gpuSrc, CV_BGR2BGRA);
        cv::cuda::meanShiftFiltering(gpuSrc, gpuDest, 30, 5);
        gpuDest.download(dst);
    #endif
    */
}

/// Uncomment this if you want to detect colors using trackbars
/*
const cv::String window_detection_name = "After Color Filtering";

const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

static void on_low_H_thresh_trackbar(int, void *) {
    low_H = cv::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *) {
    high_H = cv::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *) {
    low_S = cv::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *) {
    high_S = cv::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *) {
    low_V = cv::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *) {
    high_V = cv::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}

void createTrackbars_filter() {
    /// Create trackbars and insert them into window to change H,S,V values
    cv::namedWindow(window_detection_name);

    /// Trackbars to set thresholds for HSV values
    cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
}
*/

/// Uncomment this if you want to apply morphology using trackbars
static void on_Trackbar(int, void *) {};
/*
int dilation_size = 0;
int erosion_size = 0;
int erosion_elem = 0;
int dilation_elem = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

const cv::String window_erosion_name = "After Erosion";
const cv::String window_dilation_name = "After Dilation";

void createTrackbars_morphology() {
    /// Create windows for trackbars
    cv::namedWindow(window_erosion_name);
    cv::namedWindow(window_dilation_name);

    /// Create Erosion Trackbar
    cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_erosion_name,
                       &erosion_elem, max_elem,
                       on_Trackbar);

    cv::createTrackbar("Kernel size:\n n +1", window_erosion_name,
                       &erosion_size, max_kernel_size,
                       on_Trackbar);

    /// Create Dilation Trackbar
    cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_dilation_name,
                       &dilation_elem, max_elem,
                       on_Trackbar);

    cv::createTrackbar("Kernel size:\n 2n +1", window_dilation_name,
                       &dilation_size, max_kernel_size,
                       on_Trackbar);

}
*/

/// Uncomment this if you want to use mouse color picker (it finds color range of an object which you clicked at)
/*
cv::Mat image_for_color_pick_1;
cv::Mat image_for_color_pick_2;

/// Mouse callback function (returns color of the place where you clicked)
void pick_color(int event, int x, int y, int f, void *)
{
    if (event==1)
    {
        int r = 3;
        int off[9*2] = {0,0, -r,-r, -r,0, -r,r, 0,r, r,r, r,0, r,-r, 0,-r};
        for (int i=0; i < 9; i++)
        {
            cv::Vec3b p = image_for_color_pick_1.at<cv::Vec3b>(y+off[2*i], x+off[2*i+1]);
            std::cerr << int(p[0]) << " " << int(p[1]) << " " << int(p[2]) << std::endl;
            image_for_color_pick_2.push_back(p);
        }
    }
}
*/

void CLAHE_correction(const cv::Mat &src, cv::Mat &dst) {

    cv::Mat lab_image;
    cv::cvtColor(src, lab_image, CV_BGR2Lab);

    /// Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // Now we have the L image in lab_planes[0]

    /// Apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    clahe->apply(lab_planes[0], dst);

    /// Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    /// Convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

    dst = image_clahe;
}

void MatDetector::extractGreenColour(const cv::Mat &src, cv::Mat &dst) {
    cv::blur(src, dst, cv::Size(3,3));
    //cv::GaussianBlur(src, dst, cv::Size(3, 3), 0, 0);

    /// Uncomment this if you want to use CLAHE correction
    //CLAHE_correction(dst, dst);

    cv::Mat hsv;

    /// Convert from BGR to HSV colorspace
    cv::cvtColor(dst, hsv, CV_BGR2HSV);

    //cv::namedWindow("Filtering src");
    //if (!hsv.empty()) cv::imshow("Filtering src", hsv);

    /// This (using color picker function)
    /*
    image_for_color_pick_1 = hsv;

    cv::namedWindow("Pick Green");
    if (!src.empty()) cv::imshow("Pick Green", src);

    cv::setMouseCallback("Pick Green", pick_color, 0);

    cv::Scalar m, v;
    cv::meanStdDev(image_for_color_pick_2, m, v);
    std::cerr << "mean, var (GREEN): " << std::endl;
    std::cerr << m[0] << " " << m[1] << " " << m[2] << " " << v[0] << " " << v[1] << " " << v[2] << std::endl;

    cv::Scalar lower_green(m[0]-v[0], m[1]-v[1], m[2]-v[2]); /// Mean - var for low
    cv::Scalar higher_green(m[0]+v[0], m[1]+v[1], m[2]+v[2]); /// Mean + var for high
    */

    // TODO change the range (add blue??)
    /*
    cv::Scalar lower_green(78, 95, 0); /// Mean - var for low
    cv::Scalar higher_green(180, 255, 146); /// Mean + var for high
    */

    cv::Scalar lower_green(lower_green_H, lower_green_S, lower_green_V); /// Mean - var for low
    cv::Scalar higher_green(higher_green_H, higher_green_S, higher_green_V); /// Mean + var for high

    cv::Scalar lower_red_1(0, 70, 50); /// Mean - var for low
    cv::Scalar higher_red_1(10, 255, 255); /// Mean + var for high

    cv::Scalar lower_red_2(170, 70, 50); /// Mean - var for low
    cv::Scalar higher_red_2(180, 255, 255); /// Mean + var for high


    /**
    mean, var (RED):
    90.1111 160.963 159.333 57.1666 59.4082 51.9679
    */

    /// Detect the object based on HSV Range Values
    cv::Mat1b mask1, mask2, mask3;
    cv::Mat src_copy;

    cv::inRange(hsv, lower_green, higher_green, mask1);
    cv::inRange(hsv, lower_red_1, higher_red_1, mask2);
    cv::inRange(hsv, lower_red_2, higher_red_2, mask3);

    cv::Mat1b mask = mask1 | mask2 | mask3;


    /// Or This (using trackbars)

    //createTrackbars_filter();
    //std::array<int, 3> lower_green = {low_H, low_S, low_V}; //H_S_V
    //std::array<int, 3> higher_green = {high_H, high_S, high_V};


    /// Detect the object based on HSV Range Values
    //cv::Mat mask;
    cv::inRange(hsv, lower_green, higher_green, mask);
    cv::bitwise_and(hsv, hsv, dst, mask = mask); /// dst - image with applied mask

    dst = mask;
    //if (!dst.empty()) cv::imshow(window_detection_name, dst);
    //if (!dst.empty()) {cv::namedWindow("After Color Filtering"); cv::imshow("After Color Filtering", dst);}
}

void MatDetector::extractValueChannel(const cv::Mat &src, cv::Mat &dst) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    dst = hsvChannels[2]; /// Image hue represented as gray scale (a bit noisy)
}

void MatDetector::morphology(const cv::Mat &src, cv::Mat &dst) {
    cv::Mat element;

    /// Uncomment this if you want to apply morphology (erosion and dilation) using trackbars
    /*
    createTrackbars_morphology();

    int erosion_type;

    if (erosion_elem == 0) {erosion_type = cv::MORPH_RECT;}
    else if (erosion_elem == 1) {erosion_type = cv::MORPH_CROSS;}
    else if (erosion_elem == 2) {erosion_type = cv::MORPH_ELLIPSE;}

    element = cv::getStructuringElement(erosion_type,
                                        cv::Size(erosion_size + 1, erosion_size+1),
                                        cv::Point(erosion_size, erosion_size));

    /// Apply the erosion operation
    cv::erode(dst, dst, element);
    if (!dst.empty()) cv::imshow(window_erosion_name, dst);
    */

    /*
    int dilation_type;

    if (dilation_elem == 0) {dilation_type = cv::MORPH_RECT;}
    else if (dilation_elem == 1) {dilation_type = cv::MORPH_CROSS;}
    else if (dilation_elem == 2) {dilation_type = cv::MORPH_ELLIPSE;}

    element = cv::getStructuringElement(dilation_type,
                                        cv::Size(2*dilation_size + 1, 2*dilation_size+1),
                                        cv::Point(dilation_size, dilation_size));

    /// Apply the dilation operation
    cv::dilate(dst, dst, element);
    if (!dst.empty()) cv::imshow(window_dilation_name, dst);
    */

    /// Apply the erosion operation
    int erosion_type = cv::MORPH_RECT;
    element = cv::getStructuringElement(erosion_type,
                                        cv::Size(1 + 1, 1 + 1),
                                        cv::Point(1, 1));
    cv::erode(src, dst, element);
    //if (!dst.empty()) {cv::namedWindow("After Morpholory"); cv::imshow("After Morpholory", dst);}

    /// Apply the dilation operation
    int dilation_type = cv::MORPH_RECT;
    element = cv::getStructuringElement(dilation_type,
                                        cv::Size(2*4 + 1, 2*4 + 1),
                                        cv::Point(4, 4));
    cv::dilate(dst, dst, element);
    //if (!dst.empty()) {cv::namedWindow("After Morpholory"); cv::imshow("After Morpholory", dst);}

    /// Apply morphologyEx
    int size = 2;
    element = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(2*size+1, 2*size+1));
    cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, element);
    if (!dst.empty()) {
        //cv::namedWindow("After morphologyEx"); cv::imshow("After morphologyEx", dst);




    }
}

/// Uncomment this if you want to use trackbars to adjust canny parameters
/*
int lowThreshold;
int ratio = 3;
int kernel_size = 3;

const cv::String window_draw_contours_name = "After Canny";

void createTrackbars() {
    /// Create trackbars and insert them into window to change H,S,V values
    cv::namedWindow(window_draw_contours_name);

    /// Trackbars to set Canny function parameters
    cv::createTrackbar("lowThreshold", window_draw_contours_name, &lowThreshold, 255, on_Trackbar);
    cv::createTrackbar("Ratio", window_draw_contours_name, &ratio, 15, on_Trackbar);
    cv::createTrackbar("Kernel Size", window_draw_contours_name, &kernel_size, 10, on_Trackbar);
}
*/

void MatDetector::thresholdAndContours(const cv::Mat& src, cv::Mat& dst, std::vector<std::vector<cv::Point>>& contours) {
    //cv::blur(src, dst, cv::Size(3,3));
    std::vector<cv::Vec4i> hierarchy;

    /// Uncomment this if you want to use trackbars to adjust canny parameters
    //createTrackbars();

    cv::Mat dst_copy;
    cv::Canny(dst, dst_copy, 1, 1*3, 3);

    cv::findContours(dst_copy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    //cv::findContours(dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
}

float MatDetectorBottomCamera::getLineSlope(const cv::Vec4f &line) {
    if (line[0] == line[2])
        return 90;
    float tan = std::abs(line[1] - line[3]) / std::abs(line[0] - line[2]);
    float arctan = std::atan(tan);
    float res = arctan * 180.0f / CV_PI;
    return res;
}

float MatDetectorBottomCamera::getLength(const cv::Vec4f &line) {
    return std::sqrt((line[2]-line[0])*(line[2]-line[0]) + (line[3]-line[1])*(line[3]-line[1]));
}

float MatDetectorBottomCamera::getDistance(float x1, float y1, float x2, float y2) {
    return std::sqrt((x1 - x2)*(x1 - x2) - (y1 - y2)*(y1 - y2));
}

/// Global for reconfigure
cv::Mat imageAfterContourDetection, imageWithAllLines, imageAfterContourDetectionDrum;

bool MatDetectorFrontCamera::getMatContour(std::vector<std::vector<cv::Point>>& contours, const cv::Mat& image) {

    cv::Rect bounding_rect;

    /// Find the convex hull object for each contour
    std::vector<std::vector<cv::Point>>hull (contours.size());
    for (int i = 0; i < contours.size(); i++) {cv::convexHull(cv::Mat(contours[i]), hull[i], false);}

    /// Draw contours + hull results
    cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black
    cv::RNG rng;
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)); /// Random colors
        cv::drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        cv::drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    /// Leave only convex hulls
    contours = hull;

    /// Show in a window
    //if (!drawing.empty()) cv::imshow("After Contour detection", drawing);
    //imageAfterContourDetection = drawing;

    // TODO add max possible contour area
    int largest_contour_index = -1;
    double largest_area = 0;

    /// Approximate contours to polygons + get bounding rects (and circles)
    //std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    //std::vector<cv::Rect> boundRect(contours.size());
    //std::vector<cv::Point2f> center(contours.size());
    //std::vector<float> radius(contours.size());

    for (int i = 0; i < contours.size(); i++) {
        //cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 5, true);
        //boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
        //cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);

        /// Finding biggest contour
        double a = cv::contourArea(contours[i],false);           /// Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;                           /// Store the index of largest contour
            bounding_rect = cv::boundingRect(contours[i]);       /// Find the bounding rectangle for biggest contour
        }
    }

    drawing = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::Scalar color(255, 100, 255);
    cv::drawContours(drawing, contours, largest_contour_index, color, 1, 8, std::vector<cv::Vec4i>(), 0); /// Draw the largest contour using previously stored index

    //if (!drawing.empty()) {cv::namedWindow("Biggest Contour"); cv::imshow("Biggest Contour", drawing);}
    imageAfterContourDetection = drawing;

    //std::cout<<largest_contour_index<<" - Biggest contour index"<<std::endl;
    //std::cout<<global_contours.size()<<" - Counters amount"<<std::endl;

    /// Draw polygonal contour + bonding rects (+ circles)
    cv::Mat drawing_approx = cv::Mat::zeros(image.size(), CV_8UC3);

    color = cv::Scalar(110, 150, 200);
    //if (largest_contour_index != -1) cv::drawContours(drawing_approx, contours_poly, largest_contour_index, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );

    color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    //if (largest_contour_index != -1) cv::rectangle(drawing_approx, boundRect[largest_contour_index].tl(), boundRect[largest_contour_index].br(), color, 2, 8, 0 );

    color = cv::Scalar(110, 100, 255);
    if (largest_contour_index != -1) cv::rectangle(drawing_approx, bounding_rect.tl(), bounding_rect.br(), color, 2, 8, 0 );

    color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    //if (largest_contour_index != -1) cv::circle(drawing_approx, center[largest_contour_index], (int)radius[largest_contour_index], color, 2, 8, 0 );


    cv::Point center_of_rect = (bounding_rect.br() + bounding_rect.tl())*0.5;
    cv::circle(drawing_approx, center_of_rect, 3, cv::Scalar(0, 0, 255));

    //if (!drawing_approx.empty()) {cv::namedWindow("After approxPolyDP"); cv::imshow("After approxPolyDP", drawing_approx);}

    //std::cout<<center_of_rect<<" - Centre coordinates"<<std::endl;

    if ((center_of_rect.x != 0) || (center_of_rect.y != 0)) {
        cv::Point center_of_rect_CC = convertToCentralCoordinates(center_of_rect, drawing_approx.size().width, drawing_approx.size().height);
        //std::cout<<center_of_rect_CC<<" - Centre coordinates in CC"<<std::endl;
    }

    if (largest_contour_index != -1) contours[0] = contours[largest_contour_index];

    //std::cout<<largest_area<<" - Largest Area"<<std::endl;

    if ((!contours.empty()) && (largest_area > 200)) {return true;} else {return false;}
}

/// Sorting function (sorts elements from small to big in one array's row)
void insertionSort(std::vector<std::vector<float>>& angle, int row, int low_ind, int high_ind) {
    int i, j;
    float key_0, key_1, key_2;
    for (i = low_ind + 1; i <= high_ind; i++)
    {
        //std::cout<<angle[1][i]<<" - Angle"<<std::endl;
        key_0 = angle[0][i];
        key_1 = angle[1][i];
        key_2 = angle[2][i];

        j = i-1;

        /** Move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        if (row == 1) {
            while (j >= 0 && angle[1][j] > key_1)
            {
                angle[0][j+1] = angle[0][j];
                angle[1][j+1] = angle[1][j];
                angle[2][j+1] = angle[2][j];

                j = j-1;
            }
        }

        if (row == 2) {
            while (j >= 0 && angle[2][j] > key_2)
            {
                angle[0][j+1] = angle[0][j];
                angle[1][j+1] = angle[1][j];
                angle[2][j+1] = angle[2][j];

                j = j-1;
            }
        }
        angle[0][j+1] = key_0;
        angle[1][j+1] = key_1;
        angle[2][j+1] = key_2;
    }
}

/// Uncomment this if you want to use trackbars to adjust FastLineDetector parameters
/*
int length_threshold = 40;
int canny_th1 = 10;
int canny_th2 = 10;
int canny_aperture_size = 3;
*/

/// Uncomment this if you want to use trackbars to adjust HoughLinesP parameters
//int threshold (20), maxLineGap (20), minLineLength (20);

/// Global Mat variables for rqt_reconfigure (sorry)
cv::Mat reconfImageAfterLineDetector;

void MatDetectorBottomCamera::detectLines(const cv::Mat& image, std::vector<cv::Vec4f>& lines) {

    cv::Mat gray = image;
    //cv::cvtColor(gray, gray, CV_BGR2GRAY); /// We use mask

    /// This (FastLineDetector)(Example here https://docs.opencv.org/3.4/d1/d9e/fld_lines_8cpp-example.html#a9)
    /*
    length_threshold	10 - Segment shorter than this will be discarded
    distance_threshold	1.41421356 - A point placed from a hypothesis line segment farther than this will be regarded as an outlier
    canny_th1	50 - First threshold for hysteresis procedure in Canny()
    canny_th2	50 - Second threshold for hysteresis procedure in Canny()
    canny_aperture_size	3 - Aperturesize for the sobel operator in Canny()
    do_merge	false - If true, incremental merging of segments will be perfomred
    */

    /// Uncomment this if you want to use trackbars to adjust FastLineDetector parameters
    /*
    cv::createTrackbar("length_threshold", "All lines after FastLineDetector", &length_threshold, 500, on_Trackbar);
    cv::createTrackbar("canny_th1", "All lines after FastLineDetector", &canny_th1, 100, on_Trackbar);
    cv::createTrackbar("canny_th2", "All lines after FastLineDetector", &canny_th2, 100, on_Trackbar);
    cv::createTrackbar("canny_aperture_size", "All lines after FastLineDetector", &canny_aperture_size, 20, on_Trackbar);

    cv::Ptr<cv::ximgproc::FastLineDetector> detector = cv::ximgproc::createFastLineDetector(length_threshold, 3, canny_th1, canny_th2, canny_aperture_size, false);
    */

    //cv::Ptr<cv::ximgproc::FastLineDetector> detector = cv::ximgproc::createFastLineDetector(60, 6, 50, 50, 3, false);
    cv::Ptr<cv::ximgproc::FastLineDetector> detector = cv::ximgproc::createFastLineDetector(length_threshold, distance_threshold, 50, 50, 3, false);

    if (!gray.empty()) detector->detect(gray, lines);

    //cv::Mat drawing(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); /// White
    cv::Mat drawing(image.size(), CV_8UC3, cv::Scalar(0, 0, 0)); /// Black

    std::vector<std::vector<float>> angle(4, std::vector<float>(lines.size())); // Array for slopes

    for (int i = 0; i < lines.size(); i++) {
        /// Draw the lines
        cv::Vec4f l = lines[i];
        cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        cv::line(drawing, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 2, CV_AA);
    }

    reconfImageAfterLineDetector = drawing;

    /// Or this (HoughLinesP)
    /*
    cv::createTrackbar("threshold", "All lines after Hough transform", &threshold, 400, on_Trackbar);
    cv::createTrackbar("maxLineGap", "All lines after Hough transform", &maxLineGap, 400, on_Trackbar);
    cv::createTrackbar("minLineLength", "All lines after Hough transform", &minLineLength, 400, on_Trackbar);
    if (!gray.empty()) { cv::HoughLinesP(gray, lines, 1, CV_PI / 180, threshold, maxLineGap, minLineLength); }
    */

    //if (lines.size() != 0) std::cout << lines.size() << " - Number of lines" << std::endl;
}

cv::Mat MatDetector::getLinesImage() {
    return reconfImageAfterLineDetector;
};

cv::Mat MatDetector::getimageAfterContourDetection() {
    return imageAfterContourDetection;
}

int line_number = 100;
int parallel_criteria = 0;

std::vector<std::vector<float>> MatDetectorBottomCamera::drawAndSortLines(cv::Mat& image, std::vector<cv::Vec4f>& lines) {

    //cv::namedWindow("All lines after Hough transform");
    //cv::namedWindow("All lines after FastLineDetector");
    cv::Mat drawing(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); /// White
    //cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black

    std::vector<std::vector<float>> angle(4, std::vector<float>(lines.size())); // Array for slopes

    for (int i = 0; i < lines.size(); i++) {
        /// Draw the lines
        cv::Vec4f l = lines[i];
        cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        cv::line(drawing, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 2, CV_AA);

        cv::Vec4i vertical_line(0, 0, 0, 5);
        cv::Vec4i horizontal_line(0, 0, 5, 0);

        float slope = getLineSlope(lines[i]);
        cv::Point2f pt;
        bool is_vertical (false), is_horizontal (false);
        if (((slope >= 0) && (slope <= 45)) || ((slope >= 135) && (slope <= 180))) {pt = computeIntersect(lines[i], vertical_line); is_horizontal = true;}
        else /* ((slope > 45) && (slope < 135)) */ {pt = computeIntersect(lines[i], horizontal_line); is_vertical = true;}

        angle [3][i] = 0; // Just in case

        /// Store slopes and intersections for later use
        if ((lines.size() < line_number) && (lines.size() > 0)) {
            angle [0][i] = i;
            angle [1][i] = slope;
            if (is_horizontal) angle [2][i] = pt.y;
            else angle [2][i] = pt.x;
        }
    }

    if (lines.size() < line_number) {
        insertionSort(angle, 1, 0, lines.size() - 1); /// Sort slopes (from small to big ones)
    }

    /// Sort coordinate difference between parallel lines (from small to big ones) (for each parallel group)
    /// Dont't ask me why i did that (i tried to later make new array with no matching lines [e.g. i could exclude lines by comparing their intersections with horizontal/vertical lines])
    /* In most cases it works but sometimes not (have no idea why). So i desided to comment it for now (to make code more robust)

    int counter = 0;
    int parallel_lines_group_counter = 0;

    for (int i = 1; i < lines.size(); i++) {
        if (abs(angle [1][i-1] - angle [1][i]) == parallel_criteria) { /// Tolerance
            counter ++;
        }
        else if (counter > 0) {
            std::cerr<<counter<<" - Truly parallel lines"<<"; index = "<<i<<std::endl;
            insertionSort(angle, 2, i - counter - 1, i - 1);

            for (int j = i - counter; j <= i - 1; j++) {
                if (abs(abs(angle [2][j-1]) - abs(angle [2][j])) < 20) { /// Tolerance
                    parallel_lines_group_counter ++;
                }
                else if (parallel_lines_group_counter > 0) {
                    for (int k = j - parallel_lines_group_counter - 1; k <= j - 1; k++) {
                        angle [3][k] = 1;
                    }
                    parallel_lines_group_counter = 0;
                }
                if ((parallel_lines_group_counter > 0) && (j == i)) {
                    for (int k = j - parallel_lines_group_counter; k <= j; k++) {
                        angle [3][k] = 1;
                    }
                    parallel_lines_group_counter = 0;
                }
            }
            counter = 0;
        }

        if ((i == lines.size() - 1) && (counter > 0) && (lines.size() != 0)) {
            std::cerr<<counter<<" - Truly parallel lines"<<"; index = "<<i<<std::endl;
            insertionSort(angle, 2, i - counter, i);

            for (int j = i - counter; j <= i - 1; j++) {
                if (abs(abs(angle [2][j-1]) - abs(angle [2][j])) < 20) { /// Tolerance
                    parallel_lines_group_counter ++;
                }
                else if (parallel_lines_group_counter > 0) {
                    for (int k = j - parallel_lines_group_counter - 1; k <= j - 1; k++) {
                        angle [3][k] = 1;
                    }
                    parallel_lines_group_counter = 0;
                }
                if ((parallel_lines_group_counter > 0) && (j == i)) {
                    for (int k = j - parallel_lines_group_counter; k <= j; k++) {
                        angle [3][k] = 1;
                    }
                    parallel_lines_group_counter = 0;
                }
            }
            counter = 0;
        }
    }

    */

    /// Print array
    /*
    if ((lines.size() < line_number) && (lines.size() > 0)) {
        for (int i = 0; i <= 2; i++) {
            for (int j = 0; j < lines.size(); j++)
            {
                std::cout<<angle [i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }
    */

    //if (!drawing.empty()) cv::imshow("All lines after Hough transform", drawing);
    //if (!drawing.empty()) cv::imshow("All lines after FastLineDetector", drawing);

    imageWithAllLines = drawing;
    //image = drawing;
    return (angle);
}

cv::Mat MatDetectorBottomCamera::getimageWithAllLines() {
    return imageWithAllLines;
}

std::vector<cv::Vec4f> MatDetectorBottomCamera::findHorizontalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines) {

    float min_angle = 1000;
    int counter_horizontal = 0;
    int index_paral = 0;

    cv::Mat drawing(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); /// White

    /// Uncomment if you checked matching lines before
    /*

    int flag = 0;
    int shift = 0;

    std::vector<std::vector<float>> array_noMatching_lines(2, std::vector<float>(lines.size()));

    array_noMatching_lines [0][0] = angle [0][0];
    array_noMatching_lines [1][0] = angle [1][0];

    for (int i = 1; i < lines.size(); i++) {
        if (((angle [3][i] == 1) && (angle [3][i-1] == 0)) || ((angle [3][i] == 0) && (angle [3][i-1] == 1)) || ((angle [3][i] == 0) && (angle [3][i-1] == 0))) {
            if (flag != 0) shift = shift + flag;
            array_noMatching_lines [0][i-shift] = angle [0][i];
            array_noMatching_lines [1][i-shift] = angle [1][i];
            flag = 0;
        }

        if ((angle [3][i] == 1) && (angle [3][i-1] == 1)) {
            flag++;
            if (getLength(lines[angle [0][i]]) > getLength(lines[angle [0][i-1]])) {
                array_noMatching_lines [0][i-flag-shift] = angle [0][i];
                array_noMatching_lines [1][i-flag-shift] = angle [1][i];
            }
            else {
                array_noMatching_lines [0][i-flag-shift] = angle [0][i-1];
                array_noMatching_lines [1][i-flag-shift] = angle [1][i-1];
            }
        }
    }
    */

    /// Print array
    /*
    if ((lines.size() < line_number) && (lines.size()-shift > 0)) {
        for (int i = 0; i <= 1; i++) {
            for (int j = 0; j <= lines.size()-1-shift; j++)
            {
                std::cout<<array_noMatching_lines [i][j]<<" ";
            }
            std::cout<<std::endl;
        }
    }

    for (int i = lines.size()-1-shift; i >= 0; i--) ....
    */

    /// Find Horizontal lines
    for (int i = lines.size() - 1; i >= 0; i--) {
        if ((!lines.empty()) && ((angle [1][i] < min_angle) || (abs(abs(angle [1][i]) - abs(min_angle)) == parallel_criteria)) && (angle [1][i] < min_angle_criteria) && (getLength(lines[angle [0][i]]) > (image.cols/3))) {
                min_angle = angle [1][i];
                std::cout<<min_angle<<" - min Angle"<<std::endl;
                index_paral = i;

                counter_horizontal = 1;
        }

        /// If they are parallel
        else if ((!lines.empty()) && (abs(abs(angle [1][i]) - abs(min_angle)) == parallel_criteria) && (getLength(lines[angle [0][i]]) > (image.cols/4))) {
            counter_horizontal++;
        }
    }

    //std::cout<<counter_horizontal<<"; "<<index_paral<<" - Number of parallel horizontal lines + index"<<std::endl;

    std::vector<cv::Vec4f> horizontal_lines(counter_horizontal);

    int cntr = 0;

    /// Expand the lines
    if ((min_angle < 8) && (counter_horizontal != 0)) {
        for (int i = index_paral - counter_horizontal + 1; i <= index_paral; i++) {
            //cv::Vec4f v = lines[angle[0][i]];
            //lines[i][0] = 0;
            //lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
            //lines[i][2] = image.cols;
            //lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (image.cols - v[2]) + v[3];
            //cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
            //cv::line(drawing, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), color, 2, CV_AA);

            /// Store horizontal lines
            horizontal_lines[cntr] = lines[angle[0][i]];
            cntr++;
        }
    }

    /// Uncomment if you checked matching lines before
    /*
    cv::namedWindow("All lines after FastLineDetector (only good one)");
    cv::Mat drawing_fix(image.size(), CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i <= lines.size()-1-shift; i++) {
        /// Draw the lines
        cv::Vec4f l = lines[array_noMatching_lines[0][i]];
        cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
        cv::line(drawing_fix, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 2, CV_AA);
    }

    cv::imshow("All lines after FastLineDetector (only good one)", drawing_fix);
    */

    //cv::namedWindow("Expanded horizontal lines");
    //if (!drawing.empty()) cv::imshow("Expanded horizontal lines", drawing);
    return horizontal_lines;
}

std::vector<cv::Vec4f> MatDetectorBottomCamera::findVerticalLines(const cv::Mat& image, std::vector<std::vector<float>>& angle, std::vector<cv::Vec4f>& lines) {

    float max_angle = 0;
    int counter_vertical = 0;
    int index_paral = 0;

    cv::Mat drawing(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)); /// White

    /// Find Vertical lines

    for (int i = 0; i <= lines.size()-1; i++) {
        if ((!lines.empty()) && ((angle [1][i] > max_angle) || (abs(abs(angle [1][i]) - abs(max_angle)) == parallel_criteria)) && (angle [1][i] > max_angle_criteria) && (getLength(lines[angle [0][i]]) > (image.rows/3))) {
                max_angle = angle [1][i];
                std::cout<<max_angle<<" - max Angle"<<std::endl;
                index_paral = i;

                counter_vertical = 1;
        }

        /// If they are parallel
        else if ((!lines.empty()) && (abs(abs(angle [1][i]) - abs(max_angle)) == parallel_criteria) && (getLength(lines[angle [0][i]]) > (image.rows/2))) {
            counter_vertical++;
        }
    }

    //std::cout<<counter_vertical<<"; "<<index_paral<<" - Number of parallel vertical lines + index"<<std::endl;std::cout<<counter_vertical<<"; "<<index_paral<<" - Number of parallel vertical lines + index"<<std::endl;

    int cntr = 0;

    std::vector<cv::Vec4f> vertical_lines(counter_vertical);

    /// Expand the lines
    if ((max_angle > 80) && (counter_vertical != 0)) {
        for (int i = index_paral; i <= index_paral + counter_vertical - 1; i++) {
            //cv::Vec4f v = lines[angle[0][i]];
            //lines[i][0] = 0;
            //lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
            //lines[i][2] = image.cols;
            //lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (image.cols - v[2]) + v[3];
            //cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
            //cv::line(drawing, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), color, 2, CV_AA);

            /// Store vertical lines
            vertical_lines[cntr] = lines[angle[0][i]];
            cntr++;
        }
    }

    //cv::namedWindow("Expanded vertical lines");
    //if (!drawing.empty()) cv::imshow("Expanded vertical lines", drawing);
    return vertical_lines;
}

void MatDetector::detectContours(const cv::Mat& src, cv::Mat& image, std::vector<std::vector<cv::Point>>& contours, bool withPreprocess) {

    if (withPreprocess) {
        extractGreenColour(src, image);
        morphology(image, image);

        //defaultPreprocess(src, image);
    }
    else
        image = src.clone();

    thresholdAndContours(src, image, contours);
}

MatDescriptorFrontCamera MatDetectorFrontCamera::detect(const cv::Mat& src, cv::Mat& image, std::vector<std::vector<cv::Point>>& contours) {

    if (getMatContour(contours, image)) return MatDescriptorFrontCamera::create(contours);
    else return MatDescriptorFrontCamera::noMat();
}

cv::Mat imageAfterMaskDrum;

cv::Mat1b FrontCameraDrumDetector::getMask(const cv::Mat& src) {

    cv::Scalar lower_red_1(0, 70, 50); /// Mean - var for low
    cv::Scalar higher_red_1(10, 255, 255); /// Mean + var for high

    cv::Scalar lower_red_2(170, 70, 50); /// Mean - var for low
    cv::Scalar higher_red_2(180, 255, 255); /// Mean + var for high

    cv::Scalar lower_blue(100, 35, 50); /// Mean - var for low
    cv::Scalar higher_blue(180, 255, 255); /// Mean + var for high


    /// Detect the object based on HSV Range Values
    cv::Mat1b mask1, mask2, mask3;
    cv::Mat src_copy;

    cv::Mat hsv;

    /// Convert from BGR to HSV colorspace
    cv::cvtColor(src, hsv, CV_BGR2HSV);

    //cv::inRange(hsv, lower_blue, higher_blue, mask1); /// UNABLE THIS MB
    cv::inRange(hsv, lower_red_1, higher_red_1, mask2);
    cv::inRange(hsv, lower_red_2, higher_red_2, mask3);

    cv::Mat1b mask = mask2 | mask3;
    imageAfterMaskDrum = mask;

    return mask;
}

std::vector<std::vector<cv::Point>> FrontCameraDrumDetector::getDrumContour(const cv::Mat1b& image) {

    std::vector<cv::Vec4i> hierarchy;

    /// Uncomment this if you want to use trackbars to adjust canny parameters
    //createTrackbars();

    cv::Mat dst_copy;
    cv::Canny(image, dst_copy, 1, 1*3, 3);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dst_copy, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC3); /// Black
    cv::RNG rng;
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)); /// Random colors
        cv::drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        //cv::drawContours(drawing, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    imageAfterContourDetectionDrum = drawing;

    // TODO add max possible contour area
    int largest_contour_index = -1;
    int largest_contour_index_prev = -1;
    double largest_area = 0;

    for (int i = 0; i < contours.size(); i++) {

        /// Finding biggest contour
        double a = cv::contourArea(contours[i],false);           /// Find the area of contour
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index_prev = largest_contour_index;
            largest_contour_index = i;                           /// Store the index of largest contour
        }
    }

    drawing = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::Scalar color(255, 100, 255);
    if (largest_contour_index != -1) cv::drawContours(drawing, contours, largest_contour_index, color, 1, 8, std::vector<cv::Vec4i>(), 0); /// Draw the largest contour using previously stored index
    if (largest_contour_index_prev != -1) cv::drawContours(drawing, contours, largest_contour_index_prev, color, 1, 8, std::vector<cv::Vec4i>(), 0); /// Draw the largest contour using previously stored index

    std::vector<std::vector<cv::Point>> contour_copy = contours;

    if (!contour_copy.empty() && (largest_contour_index != -1) && (largest_contour_index_prev != -1)) {
        contour_copy[0] = contours[largest_contour_index];
        contour_copy[1] = contours[largest_contour_index_prev];
        return contour_copy;
    }
    return contour_copy;

}

cv::Mat FrontCameraDrumDetector::getimageAfterContourDetectionDrum() {
    return imageAfterContourDetectionDrum;
}

cv::Mat FrontCameraDrumDetector::getimageAfterMaskDrum() {
    return imageAfterMaskDrum;
}

FrontCameraDrumDescriptor FrontCameraDrumDetector::detect(const cv::Mat& src, cv::Mat& image) {

    cv::Mat1b mask;
    std::vector<std::vector<cv::Point>> contours;

    if (!src.empty()) {
        mask = getMask(src);}
    //morphology(mask, mask);
    if (!mask.empty())
        contours = getDrumContour(mask);
    if (contours.empty()) return FrontCameraDrumDescriptor::noDrum();
    else return FrontCameraDrumDescriptor::create(contours);
}

MatDescriptorBottomCamera MatDetectorBottomCamera::detect(const cv::Mat& src, cv::Mat &image) {

    std::vector<cv::Vec4f> lines;

    detectLines(image, lines);

    if (!lines.empty()) {

        std::vector<std::vector<float>> angle(4, std::vector<float>(lines.size())); /// Array for slopes

        angle = drawAndSortLines(image, lines);

        std::vector<cv::Vec4f> horizontal_lines = findHorizontalLines(image, angle, lines);
        std::vector<cv::Vec4f> vertical_lines = findVerticalLines(image, angle, lines);

        //std::cout<<horizontal_lines.size()<<" - Horizontal Lines"<<std::endl;
        //std::cout<<vertical_lines.size()<<" - Vertical Lines"<<std::endl;

        if ((horizontal_lines.size() == 0) && (vertical_lines.size() == 0)) return MatDescriptorBottomCamera::noLines();
        else {
            return MatDescriptorBottomCamera::create(horizontal_lines, vertical_lines);
        }
    }
    else return MatDescriptorBottomCamera::noLines();
}

/// For Dynamic Reconfigure
/// GREEN H channeldistance_threshold
float MatDetector::setLowerGreenH() const {
    return lower_green_H;
}
void MatDetector::setLowerGreenH(float lower_green_H) {
    MatDetector::lower_green_H = lower_green_H;
}


float MatDetector::setHigherGreenH() const {
    return higher_green_H;
}
void MatDetector::setHigherGreenH(float higher_green_H) {
    MatDetector::higher_green_H = higher_green_H;
}

/// GREEN S channel
float MatDetector::setLowerGreenS() const {
    return lower_green_S;
}
void MatDetector::setLowerGreenS(float lower_green_S) {
    MatDetector::lower_green_S = lower_green_S;
}


float MatDetector::setHigherGreenS() const {
    return higher_green_S;
}
void MatDetector::setHigherGreenS(float higher_green_S) {
    MatDetector::higher_green_S = higher_green_S;
}

/// GREEN V channel
float MatDetector::setLowerGreenV() const {
    return lower_green_V;
}
void MatDetector::setLowerGreenV(float lower_green_V) {
    MatDetector::lower_green_V = lower_green_V;
}


float MatDetector::setHigherGreenV() const {
    return higher_green_V;
}
void MatDetector::setHigherGreenV(float higher_green_V) {
    MatDetector::higher_green_V = higher_green_V;
}
/**********/ /**********/ /**********/ /**********/


/// For line detector
float MatDetectorBottomCamera::setLengthThreshold() const {
    return length_threshold;
}
void MatDetectorBottomCamera::setLengthThreshold(float length_threshold) {
    MatDetectorBottomCamera::length_threshold = length_threshold;
}

float MatDetectorBottomCamera::setDistanceThreshold() const {
    return distance_threshold;
}
void MatDetectorBottomCamera::setDistanceThreshold(float distance_threshold) {
    MatDetectorBottomCamera::distance_threshold = distance_threshold;
}


/// max_angle
float MatDetectorBottomCamera::setMinAngleCriteria() const {
    return min_angle_criteria;
}
void MatDetectorBottomCamera::setMinAngleCriteria(float min_angle_criteria) {
    MatDetectorBottomCamera::min_angle_criteria = min_angle_criteria;
}

/// min_angle
float MatDetectorBottomCamera::setMaxAngleCriteria() const {
    return max_angle_criteria;
}
void MatDetectorBottomCamera::setMaxAngleCriteria(float max_angle_criteria) {
    MatDetectorBottomCamera::max_angle_criteria = max_angle_criteria;
}