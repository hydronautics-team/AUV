#include "../../include/tile/TileAngle.h"

void TileAngle::imgProc(cv::Mat &img) //обработка изображения
{
    cvtColor(img,img,cv::COLOR_BGR2GRAY); // frame color to grayscale
    cv::blur(img, img, cv::Size(9,9));
    cv::adaptiveThreshold(img, img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 2);
    cv::Canny( img, img, 200, 600, 3);
}

float TileAngle::findVertical(cv::Mat &img)
{
    std::vector<float> temp_theta;
    std::vector<cv::Vec2f> lines;
    float average_sum = 0, average = 0, vertical = 0, l_corn_theta, r_corn_theta;
    int average_counter = 0;

    cv::HoughLines(img, lines, 1, CV_PI/180, 100);

    for (size_t i = 0; i < lines.size(); i++){ // находим линию не близкую к границам
        l_corn_theta = lines[i][1] - delta_theta; //дельты ро и тета
        r_corn_theta = lines[i][1] + delta_theta;
        //проверка на выход за границы
        if ((l_corn_theta < 0) | (r_corn_theta > CV_PI)) continue;
        else break;
    }
    for (size_t i = 0; i < lines.size(); i++){
        float theta = lines[i][1]; // текущие значения
        if ((theta > l_corn_theta) and (theta < r_corn_theta)){
            temp_theta.push_back(theta);
            average_sum += theta;
            average_counter++;
        }
    }
    average = average_sum/average_counter; //average

    if ((average > 0.785) and (average < 2.356)) vertical = average - CV_PI/2; // проверка на вертикальность
    else if (average > 2.356) vertical = average - CV_PI;
    else vertical = average;

    return vertical;
}

void TileAngle::angleVar(float cur, float* prev, int* quart)
{
    if (((cur > 0) & (*prev > 0)) | ((cur < 0) & (*prev < 0))){
        *prev = cur;
    }
    else{
        float delta = cur - *prev;
        if (std::fabs(delta) < 0.35) *prev = cur;
        else if (delta > 0){
            *prev = cur;
            *quart -= 1;
        }
        else{
            *prev = cur;
            *quart += 1;
        }
    }
}

void TileAngle::processing(cv::Mat &img)
{
    imgProc(img); //image processing
    angleVar(findVertical(img), &angle, &quartcircle);

    std::string angle_text;
    //vertical
    angle_text = std::to_string(angle);
    cv::putText(img,angle_text,cv::Point(10,30),CV_FONT_NORMAL,1,255);
    //global angle
    angle_text = std::to_string(angle * 180 / CV_PI + quartcircle * 90);
    cv::putText(img,angle_text,cv::Point(10,70),CV_FONT_NORMAL,1,255);
}

float TileAngle::globalAngle()
{
    return angle * 180 / CV_PI + quartcircle * 90;
}

float TileAngle::verticalAngle()
{
    return angle;
}