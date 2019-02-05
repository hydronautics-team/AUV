#ifndef AUV_VISION_TILEANGLE_H
#define AUV_VISION_TILEANGLE_H

#include <iostream>
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

class TileAngle {

private:

	const float delta_theta = 0.35; // костанта для тетта
	float angle = 0;
	int quartcircle = 0;
    cv::Mat img;
	
    void imgProc(cv::Mat &img); //обработка изображения
    float findVertical(cv::Mat &img);
	void angleVar(float cur, float* prev, int* quart);

public:
    TileAngle() = default;
    ~TileAngle() = default;
    void processing(cv::Mat &img);
	float globalAngle();
	float verticalAngle();
};

#endif //AUV_VISION_TILEANGLE_H
