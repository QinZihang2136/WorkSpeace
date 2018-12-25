#ifndef ARMOR_H
#define ARMOR_H
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
class LED_Stick{
public:
    LED_Stick():matched(false){}

    LED_Stick(const RotatedRect& R){
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    RotatedRect rect;
    bool matched;
    int match_index;
    float match_factor;
};

class armor{
public:
    armor();
    armor(const LED_Stick& L1, const LED_Stick& L2,Mat& img);
    int get_target_x(void);
    int get_target_y(void);
    void draw_rect(Mat &img) const;
    void draw_spot(Mat &img) const;
    int get_average_intensity(const Mat& img);
    void max_match(vector<LED_Stick>& LED, const int & i, const int& j);
    bool is_suitable_size(void) const;

    LED_Stick Led_stick[2];
    int error_angle;
    Point2i center;
    Rect2i rect;
    int average_intensity;
};



void filiter_smail_rect(vector<LED_Stick>& LED);

class KalmanFilters
{
public:
    KalmanFilters();
    KalmanFilters(int x, int y) :
        KF_(4, 2)
      /*
              KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
              "dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
              "measureParams = 2": 2*1 vector of measurement (x, y)
              */
    {
        measurement_ = Mat::zeros(2, 1, CV_32F);// (x, y)
        KF_.transitionMatrix = (Mat_<float>(4, 4) <<
                                1, 0, 1, 0,//**Latter 1: Larger, faster regression
                                0, 1, 0, 1,//**Latter 1: Larger, faster regression
                                0, 0, 1, 0,
                                0, 0, 0, 1);
        setIdentity(KF_.measurementMatrix, Scalar::all(1));
        setIdentity(KF_.processNoiseCov, Scalar::all(1e-3));//**3: Larger, slower regression
        setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));//1: Larger, quicker regression
        setIdentity(KF_.errorCovPost, Scalar::all(1));

        KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//Ensure beginner is default value
    }

    Point2f run(float x, float y)
    {
        Mat prediction = KF_.predict();
        Point2f predict_pt = Point2f(prediction.at<float>(0), prediction.at<float>(1));

        measurement_.at<float>(0, 0) = x;
        measurement_.at<float>(1, 0) = y;

        KF_.correct(measurement_);

        return predict_pt;
    }
private:
    Mat measurement_;
    cv::KalmanFilter KF_;//Differ from Kalman_example::KalmanFilter
};


#endif // ARMOR_H
