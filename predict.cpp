#include <predict.h>
/*
"**" after "//" means the values are relative to Kalman regression speed.
*/

using namespace std;
using namespace cv;
float size_x;
float size_y;
float anti_range;
Point2f currentPoint;
Point2f kalmanPoint;
Point2f anti_kalmanPoint;

void kalman_init(float target_x,float target_y,Kalman_example::KalmanFilter kf)
{
    size_x = 1280;//cols of side
    size_y = 720;//rows of side

    anti_range = 1;//**Larger, anti-kalman more radical
    //Differ from cv::KalmanFilter
    currentPoint = Point2f(target_x,target_y);
    kalmanPoint = Point2f(target_x,target_y);
    anti_kalmanPoint = Point2f(target_x,target_y);
}

int kalman_predict(float target_x,float target_y,Kalman_example::KalmanFilter kf)
{
    currentPoint = Point2f(target_x, target_y);
    kalmanPoint = kf.run(target_x, target_y);

    if ((currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) <= size_x
        || (currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x)) >= 0)//Prevent Anti-kal out of Mat
    {
        if (abs(currentPoint.x - kalmanPoint.x) > 40)//When points are closed, no Anti-kalman to reduce shaking
        {
        anti_kalmanPoint.x = currentPoint.x + anti_range*(currentPoint.x - kalmanPoint.x);
        anti_kalmanPoint.y = currentPoint.y + anti_range*(currentPoint.y - kalmanPoint.y);
        }
        else
        {
            anti_kalmanPoint.x = currentPoint.x;
            anti_kalmanPoint.y = kalmanPoint.y;
        }
    }
    else
    {
        anti_kalmanPoint.x = currentPoint.x;
        anti_kalmanPoint.y = kalmanPoint.y;
    }
    return 0;
}

void draw_kalmanPoint(Mat& image)
{
    circle(image, kalmanPoint, 3, Scalar(0, 255, 0), 2);//predicted point with green
    circle(image, currentPoint, 3, Scalar(255, 255, 255), 2);//current position with red
    circle(image, anti_kalmanPoint, 3, Scalar(0, 0, 255), 2);//current position with red
    line(image, Point(currentPoint.x - 30, currentPoint.y), Point(currentPoint.x + 30, currentPoint.y), Scalar(255, 255, 255), 1);
    line(image, Point(currentPoint.x, currentPoint.y - 30), Point(currentPoint.x, currentPoint.y + 30), Scalar(255, 255, 255),1);
    line(image, Point(anti_kalmanPoint.x - 30, anti_kalmanPoint.y),Point(anti_kalmanPoint.x + 30, anti_kalmanPoint.y),Scalar(0, 0, 255),1);
    line(image, Point(anti_kalmanPoint.x , anti_kalmanPoint.y-30), Point(anti_kalmanPoint.x , anti_kalmanPoint.y + 30), Scalar(0, 0, 255), 1);
}
