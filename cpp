#include "kalmantest.h"

RM_kalmanfilter::RM_kalmanfilter()
    :S_kalman(3,3)
{
    measurement_img = Mat::zeros(3,1,CV_64F);
    t = (1e-2) + 0.005666666f;;
    S_kalman.transitionMatrix = (Mat_<float>(4, 4) << 1, t, pow(t,2)/2, 
                                                 0, 1, t, 
                                                 0, 0, 1);
                                                 

    setIdentity(S_kalman.measurementMatrix, Scalar::all(1));
    setIdentity(S_kalman.processNoiseCov, Scalar::all(0.0000000001f));
    setIdentity(S_kalman.measurementNoiseCov, Scalar::all(1e-2));//测量协方差矩阵R，数值越大回归越慢
    setIdentity(S_kalman.errorCovPost, Scalar::all(1e-5));

    S_kalman.statePost = (Mat_<float>(3,1)<< x, y, 0, 0);
}

RM_kalmanfilter::~RM_kalmanfilter(){

}

Point2f RM_kalmanfilter::point_predict(double t1, Point2d p1)
{
    t=t1;
    double v = (p1.x - last_point.x) / t;
    double a = v/ t;

    measurement_img.at < double > (0, 0) = p1.x;
    measurement_img.at < double > (1, 0) = v;
    measurement_img.at < double > (2, 0) = a;

    Mat prediction2 = S_kalman.predict(); //至此完成了对下一帧单纯计算的预测，得出的点更加平稳。如果保证测量值够准，可以直接取这里算出的点
    Mat prediction = S_kalman.correct(measurement_img); //至此完成了对下一帧的最优估计，得出的点理论上更接近真实值。同时用于迭代，得出statePost供下一帧predict计算
    Point2f temp_point = Point2f(prediction.at < double > (0, 0), p1.y);

    double temp_x = p1.x + pow(anti_range,1.6) * (p1.x - temp_point.x);
    Point2f anti_kalman_point;

    if(a<-A||a>A){
        n++;
    }
    else{n=2;}
    if(temp_x <= 640 && temp_x >= 0)
    {
        if (abs(p1.x - temp_point.x) > 0)
        {
            anti_kalman_point.x = temp_x;
        }
        else
        {
            anti_kalman_point.x = p1.x;
        }
    }
    else
    {
        anti_kalman_point.x = p1.x;
    }


    anti_kalman_point.y = temp_point.y;

    last_v = v;
    last_point = p1;
    return anti_kalman_point;
}
