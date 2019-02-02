#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <util/ImgprocUtil.h>
#include <common/AbstractImageConverter.h>
#include "std_msgs/Empty.h"

#include <iostream>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

// JUST FOR DEBUG! REAL TOPIC IS /cam_bottom/image_raw
static const std::string CAMERA_TOPIC = "/cam_front_1/image_raw";

static const std::string TILE_ANGLE_PUBLISH_TOPIC = "/tile/angle";

static const std::string TILE_POSITION_PUBLISH_TOPIC = "/tile/position";

static const std::string TILE_LOCATOR_NODE_NAME = "tile_locator";

class TilePublisher : public AbstractImageConverter
{

private:

    ros::Publisher anglePublisher;
    ros::Publisher positionPublisher;

    ros::Subscriber resetSubscriber;

    // Stub values
    int positionX = 0;
    int positionY = 0;

    float angle = 0;
    int quartcircle = 0;
    const float delta_theta = 0.35; // костанта для тетта
    const float delta_rho = 20; // костанта для ро

    void imgProc(Mat img) //обработка изображения
    {
        blur(img, img, Size(9,9));
        adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, 2);
        Canny( img, img, 200, 600, 3);
    }

    float findVertical(Mat img)
    {
        vector<float> temp_theta;
        vector<Vec2f> lines;
        float average_sum = 0, average = 0, vertical = 0, l_corn_theta, r_corn_theta;
        int average_counter = 0;

        HoughLines(img, lines, 1, CV_PI/180, 100);

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

    void angleVar(float cur, float* prev, int* quart)
    {
        if (((cur > 0) & (*prev > 0)) | ((cur < 0) & (*prev < 0))){
            *prev = cur;
        }
        else{
            float delta = cur - *prev;
            if (fabs(delta) < 0.35) *prev = cur;
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


protected:

    // Stub logic
    void process(const cv_bridge::CvImagePtr& cv_ptr)
    {
        cv::Mat frame = cv_ptr->image;
        string angle_text;

        cvtColor(frame,frame,COLOR_BGR2GRAY); // frame color to grayscale
        imgProc(frame); //image processing
        //vertical
        //angle_text = std::to_string(findVertical(frame) * 180 / CV_PI);
        //putText(frame,angle_text,Point(10,110),CV_FONT_NORMAL,1,255);
        //global angle
        angleVar(findVertical(frame), &angle, &quartcircle);
        //angle_text = std::to_string(angle * 180 / CV_PI + quartcircle * 90);
        //putText(frame,std::to_string(angle * 180 / CV_PI),Point(10,30),CV_FONT_NORMAL,1,255);
        //putText(frame,std::to_string(quartcircle*45),Point(10,70),CV_FONT_NORMAL,1,255);
        //putText(frame,angle_text,Point(10,110),CV_FONT_NORMAL,1,255);


        std_msgs::Float32 angleMsg;
        angleMsg.data = angle + quartcircle * CV_PI;
        anglePublisher.publish(angleMsg);

        if (positionX > 2000)
            positionX = 0;
        if (positionY > 2000)
            positionY = 0;
        positionX++;
        positionY++;
        geometry_msgs::Point positionMsg;
        positionMsg.x = positionX;
        positionMsg.y = positionY;
        positionMsg.z = 0;
        positionPublisher.publish(positionMsg);
    }

public:

    TilePublisher(const std::string& inputImageTopic) : AbstractImageConverter(inputImageTopic)
    {
        anglePublisher = nodeHandle.advertise<std_msgs::Float32>(TILE_ANGLE_PUBLISH_TOPIC, 100);
        positionPublisher = nodeHandle.advertise<geometry_msgs::Point>(TILE_POSITION_PUBLISH_TOPIC, 100);
    }

    ~TilePublisher()
    {
    }

};


/* TODO Fix code style */
int main(int argc, char **argv)
{

    ros::init(argc, argv, TILE_LOCATOR_NODE_NAME);
    TilePublisher tilePublisher(CAMERA_TOPIC);

    ros::spin();

    return 0;
}