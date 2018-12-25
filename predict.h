#ifndef PREDICT_H
#define PREDICT_H
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace Kalman_example
{

class KalmanFilter
{
public:
    KalmanFilter(int x, int y) :
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

}

void kalman_init(float target_x,float target_y,Kalman_example::KalmanFilter kf);
int kalman_predict(float target_x, float target_y,Kalman_example::KalmanFilter kf);
void draw_kalmanPoint(Mat& image);
void draw_kalmanCurrent(Mat& image);
void draw_kalmanAnti(Mat& image);

#endif // PREDICT_H
