
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


class Kalman1
{
public:
    Kalman1(){
        Q_ = 0.03f;
        R_ = 0.05f;
        t_ = 1.0f;
        x_ = 0.0f;
        p_ = 0.01f;
    }
    Kalman1(float Q, float R, float t, float x0, float p0){
        Q_ = Q;
        R_ = R;
        t_ = t;
        x_ = x0;
        p_ = p0;
    }
        float run(float data){
       x_pre_ = x_;                                      //x(k|k-1) = AX(k-1|k-1)+BU(k)
        p_pre_ = p_ + Q_;                              //p(k|k-1) = Ap(k-1|k-1)A'+Q
        kg_ = p_pre_ / (p_pre_ + R_);               //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
        x_ = x_pre_ + kg_ * (data - x_pre_);          //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
        p_ = (1 - kg_) * p_pre_;                   //p(k|k) = (I-kg(k)H)P(k|k-1)
        return x_;
    }
#endif 
