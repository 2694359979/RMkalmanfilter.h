#ifndef KALMANTEST_H
#define KALMANTEST_H
#define A 0.000001f
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include<iostream>
using namespace cv;
using namespace std;

class RM_kalmanfilter
{
public:
    RM_kalmanfilter();
    ~RM_kalmanfilter();
    Point2f point_predict(double _t1, Point2d _p1);
    void reset();
    int n=2;
    float anti_range = 1.5;

private:
    cv::KalmanFilter S_kalman;
    Mat measurement_img;
    Point2f last_point;
    double last_v=0;
    double t =(1e-2) + 0.005666666f;
    float x = 640;
    float y = 400;
};

#endif 
