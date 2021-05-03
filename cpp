#include "kalmantest.h"

RM_kalmanfilter::RM_kalmanfilter()
    :S_kalman(4,2)
{
    measurement_img = Mat::zeros(2,1,CV_64F);
    t = (1e-2) + 0.005666666f;;
    S_kalman.transitionMatrix = (Mat_<float>(4, 4) << 1, t, pow(t,2)/2,0,
                                                      0, 1, t,pow(t,2)/2 
                                                      0, 0, 1,0,
                                                      0, 0, 0,1);

    setIdentity(S_kalman.measurementMatrix, Scalar::all(1));//H_k观测矩阵，为1就可以
    setIdentity(S_kalman.processNoiseCov, Scalar::all(0.0000000001f));//Q_k过程噪声协方差矩阵
    setIdentity(S_kalman.measurementNoiseCov, Scalar::all(1e-2));//R_k测量噪声协方差矩阵
    setIdentity(S_kalman.errorCovPost, Scalar::all(1e-5));//P_k后验误差估计协方差矩阵

    S_kalman.statePost = (Mat_<float>(3,1)<< 0, 0, 0); //后验更新值(迭代起点)
    last_point.x = 640;
    last_point.y = 400;
}
 
Point2f RM_kalmanfilter::point_predict(double t1, Point2d p1)
{
    t=t1;
    double v = (p1.x - last_point.x) / t;
    double a = v/ t;
    measurement_img.at < double > (0, 0) = p1.x;
    measurement_img.at < double > (1, 0) = v;
    measurement_img.at < double > (2, 0) = a;

    Mat prediction2 = S_kalman.predict(); //如果影响较小，可以考虑加入一阶滤波器直接取这里算出的点
    Mat prediction = S_kalman.correct(measurement_img); 
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
//待测试(后续如果上面predict不好用，则考虑下面这个)
/*double Predictor::predict(double time){
    std::list<double>::const_iterator it_in = history_time.begin();
    double latest_value = history_value.back();

    std::list<double>::const_iterator it_out = history_value.begin();
    std::list<double>::const_iterator prev_out = it_out;
    double max_o = -500000, min_o = 500000;

    Mat A(history_size,3,CV_64F);
    Mat b(history_size,1,CV_64F);
    double * b_data = (double *) b.data;
    double * A_data = (double *) A.data;
    // Ａ矩阵 Ｂ矩阵赋值
    for (; it_in != history_time.end(); ++A_data, ++b_data, ++it_in, ++it_out){
        *A_data = (*it_in-time) * (*it_in-time);
        *(++A_data) = (*it_in-time);
        *(++A_data) = 1;
        *b_data = *it_out;
    }
    Mat A_t = A.t();
    Mat w = (A_t*A).inv()*A_t * b;//这里可以直接用单位矩阵
    Mat q = (Mat_<double>(1,3) << 0, 0, 1);
    Mat ret = q*w;

    double predict_angel = ret.at<double>(0);
    const double max_gap = 10.0;
    if(predict_angel - latest_value > max_gap)
        predict_angel = latest_value + max_gap;
    else if(predict_angel - latest_value < -max_gap)
        predict_angel = latest_value - max_gap;
    return predict_angel;
}
(考虑加入的后续稳定控制方法)
double Predictor::predict(Point predict, Point local)
{

    int error_x;  
    int error_y; 
    int dis;

    predict_x = predict.x;  /*预测位置*/
    predict_y = predict.y;

    last_x = local.x;  /*上一次的实际位置*/
    last_y = local.y;

    error_x = abs(predict_x - last_x);
    error_y = abs(predict_y - last_y);

    dis = point_dis(predict, local);


    if(dis > 5 && dis < 10)
    {  
        true_location.x = predict_x;
        true_location.y = predict_y;
    }


    if(dis > 10)  
    {
    
        error_x = error_x*KP + error_x*KI + error_x*KD;  
        error_y = error_y*KP + error_y*KI + error_y*KD;

     
        if(predict_x >= last_x && predict_y >= last_y)
        {
            true_location.x = last_x + error_x;
            true_location.y = last_y + error_y;
        }
        else if (predict_x >= last_x && predict_y <= last_y)
        {
            true_location.x = last_x + error_x;
            true_location.y = last_y - error_y;
        }
        else if (predict_x <= last_x && predict_y <= last_y)
        {
            true_location.x = last_x - error_x;
            true_location.y = last_y - error_y;
        }
        else if (predict_x <= last_x && predict_y >= last_y)
        {
            true_location.x = last_x - error_x;
            true_location.y = last_y + error_y;
        }
        else {

        }

     }
    return true_location;  
}
