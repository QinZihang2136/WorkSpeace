#include <main.h>
using namespace std;
using namespace cv;
#define IMG_ROW 1280
#define IMG_COL 720

Mat dst,gray;
int max_value = 85;
int min_vaule = 18;
void callback(int a, void*);
float range[] = { 0, 256 };
int channels[] = {0};
const float * histRanges = range;
Mat hist;
int histSize = 2;


int car_color = 1; //0->red 1->blue
float x_a, y_a, x_a_p, y_a_p;
int offset_y = 100, offset_x = 100, latency = 100;
int ksize = 5;
RotatedRect R,L;
Rect RR;
serial_transimt_date tx_data;
_serial serial("/dev/ttyUSB0");
int number=50;
double distanceX,distanceY;
Point2f currentPoint;
Point2f kalmanPoint;
Point2f anti_kalmanPoint;
KalmanFilters kf(0,0);

void onMouse( int event, int x, int y, int, void* );
int main(int argc, char** argv)
{

    float size_x = 1280;//cols of side
    float size_y = 480;//rows of side
    //float x = 20;//CV_32F: float
    //float y = 240;//CV_32F: float
    //int color = 0;//gradually varied color
    float anti_range = 1;//**Larger, anti-kalman more radical

    namedWindow("GG");
    createTrackbar("min","GG",&min_vaule,255);
    createTrackbar("max","GG",&max_value,255);

    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//设置为MJPG格式
    cap.set(CV_CAP_PROP_FRAME_WIDTH,IMG_ROW);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,IMG_COL);
    cap.set(CV_CAP_PROP_FPS,60);

    //VideoCapture cap("/home/cz/Pictures/BlueArmorVideo.avi");
    //VideoCapture cap("/home/cz/Pictures/RedArmorVideo.avi");
    if (!cap.isOpened())
    {
        printf("could not find video file");
        return -1;
    }
    namedWindow("Control", WINDOW_AUTOSIZE);

    namedWindow("Blue",WINDOW_AUTOSIZE);
    setMouseCallback( "Blue", onMouse, nullptr );
    int iLowH = 60;int iHighH = 255;
    int iLowS = 0;int iHighS = 255;
    int iLowV = 150;int iHighV = 255;
    float dx,dy;

    //double exposures = cap.get(CV_CAP_PROP_AUTO_EXPOSURE);
    //double exposure = cvGetCaptureProperty(cap,CV_CAP_PROP_EXPOSURE);

    //Create trackbars in "Control" window
    {
        cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 255);
        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);
        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
        createTrackbar("kernel_size", "Control", &ksize, 50);
        createTrackbar("offset_y", "Control", &offset_y, 200);
        createTrackbar("offset x" , "Control", &offset_x, 200 );
        createTrackbar("latency" , "Control", &latency, 200 );
    }

    int target_x,target_y;

    Mat frame,gray;
    vector<vector<Point>> contour;
    vector<Vec4i> hier;
    vector<LED_Stick> LED_Stick_v;
    bool init = false;
    cap>>frame;
    //cap.read(frame);
    Mat imgHSV;
    cvtColor(frame, imgHSV, COLOR_BGR2HSV);
    cvtColor(frame,gray,CV_BGR2GRAY);
    cout<<frame.size()<<endl;


    while (cap.read(frame))
    {
        createTrackbar("waittime","Control",&ksize,1000);
        Mat mask;
        //pre proces
        blur(frame,frame,Size(3,3));
        cvtColor(frame, imgHSV, COLOR_BGR2HSV);
        cvtColor(frame,gray,CV_BGR2GRAY);
        if(car_color){
            //blue
            //inRange(imgHSV, Scalar(61, 0 ,218), Scalar(126, 182, 255), mask); //Threshold the image
            //inRange(imgHSV,Scalar(60,0,235), Scalar(140, 255, 255),mask);//680x480
            inRange(imgHSV,Scalar(35,0,235), Scalar(117, 235, 255),mask);//680x480

            //inRange(imgHSV,Scalar(107,0,39), Scalar(135, 255, 255),mask);
        }
        else{
            //red
            Mat1b mask1, mask2;
            inRange(imgHSV, Scalar(0,0,150), Scalar(50,255,255), mask1);
            inRange(imgHSV, Scalar(150,0,150), Scalar(180,255,255), mask2);
            mask = mask1| mask2;
        }
        //inRange(imgHSV, Scalar(iLowH,iLowS,iLowV), Scalar(iHighH,iHighS,iHighV), mask);
        Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
        morphologyEx(mask,mask,MORPH_CLOSE,kernel);
        //imshow("dilate", mask);

        findContours(mask, contour, hier, RETR_TREE, CHAIN_APPROX_SIMPLE );
        LED_Stick_v.clear();
        vector<vector<Point>> convexs(contour.size());
#pragma omp for
        for(size_t i = 0; i < contour.size(); i++)
        {
            convexHull(contour[i], convexs[i], false, true);
            //approxPolyDP(Mat(contour[i]), convexs[i],4,true);
        }

#pragma omp for
        for( size_t i = 0; i< contour.size(); i++ )
        {
            if(hier.at(i)[3] != -1) continue;
            double length = arcLength( Mat(contour[i]), true);

            //LED Stick detect
            if (length > 1 && length <430)
            {
                RotatedRect RRect = minAreaRect( Mat(convexs[i]));

                //rotated Rect angle transform
                if(RRect.size.height < RRect.size.width)    // convert angle to
                {
                    RRect.angle+= 90;
                    double tmp = RRect.size.height;
                    RRect.size.height = RRect.size.width;
                    RRect.size.width = tmp;
                }
                if (abs(RRect.angle) <= 25)   // angle in range (-20,20) degree
                {
                    //draw LED stick
                    LED_Stick r(RRect);
                    LED_Stick_v.push_back(r);
                    Point2f rect_point[4];
                    RRect.points(rect_point);
#pragma omp for
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(frame, Point_<int>(rect_point[i]), Point_<int>(rect_point[(i+1)%4]), Scalar(0,255,0),2);
                    }
                }
            }
        }


        vector<armor> final_armor_list;
#pragma omp for
        for(int i = 0; i < LED_Stick_v.size(); i++)
        {
#pragma omp for
            for(int j = i + 1; j < LED_Stick_v.size(); j++)
            {
                //calc two LED stick param(center.xy,stick\rect.width,height)
                armor arm_tmp(LED_Stick_v.at(i), LED_Stick_v.at(j),frame);
                if(abs(arm_tmp.error_angle) < 8)
                {
                    if(arm_tmp.is_suitable_size())
                    {
                        if(arm_tmp.get_average_intensity(gray)<20)
                            arm_tmp.max_match(LED_Stick_v, i, j);
                        //                        target_x = arm_tmp.get_target_x();
                        //                        target_y = arm_tmp.get_target_y();
                        final_armor_list.push_back(arm_tmp);

                        //                        arm_tmp.draw_rect(frame);
                        //                        arm_tmp.draw_spot(frame);

                        // cout<< target_x<<" "<<target_y<<endl;
                        //imshow("stop",frame);
                    }
                }
            }
        }


        //DEBUG find the armor
        float dist=1e8;
        armor target;
        target.center.x = 640 + offset_x -100; // trackbar get only positive vulue;
        target.center.y = 360 + offset_y-100;
        int8_t found= 0;
        for (int i = 0; i < final_armor_list.size() ; i++ )
        {
            dx = pow((final_armor_list.at(i).center.x - 640), 2);
            dy = pow((final_armor_list.at(i).center.y - 360), 2);
            if( dx + dy < dist)
                target = final_armor_list.at(i);
            //            final_armor_list.at(i).draw_rect(frame);
            found =1;
            //            final_armor_list.at(i).draw_spot(frame);
        }


        int x=0;
        double distant;
        double anglesX,anglesY;
        double dheight;
        if(final_armor_list.size()>0)
        {
            RR = final_armor_list[0].rect;
            if(final_armor_list[0].Led_stick[0].rect.center.x>final_armor_list[0].Led_stick[1].rect.center.x)
            {
                R = final_armor_list[0].Led_stick[0].rect;
                L = final_armor_list[0].Led_stick[1].rect;
            }else
            {
                R = final_armor_list[0].Led_stick[1].rect;
                L = final_armor_list[0].Led_stick[0].rect;
            }

            //            printf("L: %4.6f,%4.6f\tR: %4.6f,%4.6f\r\n",L.size.width,L.size.height,
            //                   R.size.width,R.size.height);
            x = final_armor_list[0].rect.width;
            if(x>80&&x<300){//too close and too far
                dheight =abs(target.Led_stick[0].rect.size.height-target.Led_stick[1].rect.size.height);


                //                distant = -2e-5*pow(x,3) + 0.0111*pow(x,2)-2.8239*x+304.71;//680x480 1m-2m is ok

                dx = target.center.x - IMG_ROW/2 + offset_x - 100;
                dy = target.center.y - IMG_COL/2 + offset_y - 100;
                distanceX = dx/x*13;
                distanceY = dy/x*13;

                //target.draw_rect(frame);

                //lz
                x_a = atan(dx * tan(25/57.295) / IMG_ROW/2) * 57.295;
                y_a = atan(dy * tan(25/57.295) / IMG_COL/2) * 57.295;

                //points correction
                {
                    Point2f pts[4];
                    Point2f point[4];/*0 1
                                   3 2*/
                    L.points(pts);
                    float lheight = (pts[3].y - pts[2].y)/2;
                    float lpoint = (pts[3].y + pts[2].y) /2;
                    point[0] = Point2f(pts[2].x,lpoint-2*lheight);
                    point[3] = Point2f(pts[3].x,lpoint+2*lheight);
                    R.points(pts);
                    float rheight = (pts[0].y - pts[1].y)/2;
                    float rpoint = (pts[0].y + pts[1].y) /2;
                    point[1] = Point2f(pts[1].x,rpoint-2*rheight);
                    point[2] = Point2f(pts[0].x,rpoint+2*rheight);
//                    for(int i=0;i<4;i++)
//                    {
//                        circle(frame,point[i],3,Scalar(0,0,255),-1);
//                        line(frame,point[i],point[(i+1)%4],Scalar(255,255,0),1);
//                    }
                    //cout<<point[0].x-point[1].x<<" "<<point[0].y-point[3].y<<endl;
                    float wid=0,hei=0;
                    if(L.size.height>R.size.height)
                    {
                        wid = abs((point[0].y-point[3].y)*2.55);// 2.8=117/42  scale of width/height
                        hei = abs(point[3].y-point[0].y);
                    }else
                    {
                        wid = abs((point[1].y-point[2].y)*2.55);
                        hei = abs(point[2].y-point[1].y);
                    }
                    Point2f dst_point[4];
                    dst_point[0] = Point2f(0,0);
                    dst_point[1] = Point2f(wid,0);
                    dst_point[2] = Point2f(wid,hei*2);
                    dst_point[3] = Point2f(0,hei*2);
                    Mat warpmatrix = getPerspectiveTransform(point,dst_point);
                    Mat resultImage ;
                    warpPerspective(frame,resultImage,warpmatrix,Size(int(wid),int(hei*2)),INTER_LINEAR);
                    namedWindow("Final result",CV_WINDOW_AUTOSIZE);
                    imshow("Final result", resultImage);
                    int wegiht = resultImage.cols/4;
                    int high = resultImage.rows;
                    Rect box = Rect(Point(wegiht,0) ,Point(3*(resultImage.cols/4),high) );

                    Mat roi =resultImage(box);
                    imshow("test",roi);




                    GaussianBlur(roi,dst,Size(5,5),11);
                    cvtColor(dst,gray,COLOR_BGR2GRAY);
                    inRange(gray,min_vaule,max_value,dst);
                    calcHist(&dst,1,0,Mat(),hist,1,&histSize,&histRanges);
                    float b = 0;
                    float w = 0;

                    b =  hist.at<float>(0, 0);
                    w =  hist.at<float>(0, 1);
                    float ratio = 0;
                    ratio = b/w;
                    cout << "The ratio = " << ratio;
                    imshow("GG",dst);



                }
                target.draw_spot(frame);

                double ave = (L.size.height+R.size.height)/2; // 13/5.5

                double a = dheight/(ave*0.1);

                if(dheight>3)
                    a=0.8;
                else
                    a=0.2;

                x = (1.0-a)*x + a*ave*2.6;
                line(frame,Point(target.center.x-x/2,target.center.y),
                     Point(target.center.x+x/2,target.center.y),Scalar(255,255,255));
                distant = 6e-8*pow(x,4)-8e-5*pow(x,3)+0.0376*pow(x,2)-7.9802*x+760.63;//1280x720 1m-3.5m is ok
                anglesX = atan(distanceX/distant)*57.295; //57.295=180/CV_PI
                anglesY = atan(distanceY/distant)*57.295;

                if(init==0)
                {
                    //kf(target.center.x, target.center.y);
                    Point2f currentPoint(IMG_ROW/2,IMG_COL/2);
                    //Point2f currentPoint(target.center.x, target.center.y);
                    Point2f kalmanPoint(target.center.x, target.center.y);
                    Point2f anti_kalmanPoint(target.center.x, target.center.y);
                    init = 1;
                }else
                {
                    currentPoint = Point2f(IMG_ROW/2,IMG_COL/2);
                    //currentPoint = Point2f(target.center.x, target.center.y);
                    kalmanPoint = kf.run(target.center.x, target.center.y);
                    //kalman
                    //anti_range = float(latency)/10.0;
                    {
                    circle(frame, kalmanPoint, 3, Scalar(0, 255, 0), 2);//predicted point with green
                    circle(frame, currentPoint, 3, Scalar(255, 255, 255), 2);//current position with red
                    circle(frame, anti_kalmanPoint, 3, Scalar(0, 0, 255), 2);//current position with red
                    line(frame, Point(currentPoint.x - 30, currentPoint.y), Point(currentPoint.x + 30, currentPoint.y), Scalar(255, 255, 255), 1);
                    line(frame, Point(currentPoint.x, currentPoint.y - 30), Point(currentPoint.x, currentPoint.y + 30), Scalar(255, 255, 255),1);
                    line(frame, Point(anti_kalmanPoint.x - 30, anti_kalmanPoint.y),Point(anti_kalmanPoint.x + 30, anti_kalmanPoint.y),Scalar(0, 0, 255),1);
                    line(frame, Point(anti_kalmanPoint.x , anti_kalmanPoint.y-30), Point(anti_kalmanPoint.x , anti_kalmanPoint.y + 30), Scalar(0, 0, 255), 1);
                    }
                }
                dx = kalmanPoint.x - IMG_ROW/2 + 177 - 100;
                dy = kalmanPoint.y - IMG_COL/2 + offset_y - 100;
                x_a = atan(dx * tan(25/57.295) / IMG_ROW/2) * 57.295;
                y_a = atan(dy * tan(25/57.295) / IMG_COL/2) * 57.295;

            }
        }else
        {
            distant = 0;
            distanceX = 0;  distanceY = 0;
            anglesX = 0;    anglesY = 0;
            x_a = 0;        y_a = 0;
            init = 0;
        }

        //        //for draw
        putText(frame, "  angle x = "+to_string(anglesX)+"  y = "+ to_string(anglesY)+"  d = "+ to_string(x), Point(0,20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);
        putText(frame, "  angle x = "+to_string(x_a)+"  y = "+ to_string(y_a)+"  d = "+ to_string(dheight), Point(0,40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);
        putText(frame, "D ="+to_string(distant), Point(50,400), FONT_HERSHEY_SIMPLEX, 3, Scalar(255,255,255),3);

        imshow("Blue",frame);
        tx_data.get_xy_date(int16_t(x_a*100),int16_t(y_a), found);
        serial.send_data(tx_data);

        if(waitKey(1)>0)
            break;
    }
    close(serial.fd);
    cap.release();
    waitKey(0);
    return 0;
}

void onMouse( int event, int x, int y, int, void* )
{
    if( event ==  CV_EVENT_LBUTTONDOWN)
    {
        double x = RR.width;
        double distant = -4e-5 * pow(x,3) + 0.025 * pow(x,2) - 5.6458 * x + 476.1 + 20;
        //double distant = 2e-7*pow(x,4)-0.0002*pow(x,3)+0.0688*pow(x,2)-9.7754*x+595.7;
        //double distant = 1e-11*pow(x,6)-1e-8*pow(x,5)+7e-6*pow(x,4)-0.0017*pow(x,3)+0.2536*pow(x,2)-20.431*x+814.11;

        printf("%d  L: %4.6f,%4.6f\tR: %4.6f,%4.6f\t%d\t%4.6f\r\n",number,L.size.width,L.size.height,
               R.size.width,R.size.height,RR.width,distant);
        number += 10;

    }
    return;
}
