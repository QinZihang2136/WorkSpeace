#include "main.h"

armor::armor(){}

armor::armor(const LED_Stick& L1, const LED_Stick& L2,Mat& img)
{
    Led_stick[0] = L1;
    Led_stick[1] = L2;
  //  circle(img,L1.rect.center,3,Scalar(255,0,255),-1);
   // circle(img,L2.rect.center,3,Scalar(255,0,255),-1);
    //if error_angel is too big then the stick are not match.
    error_angle = L1.rect.angle - L2.rect.angle;
    rect.width = abs(L1.rect.center.x - L2.rect.center.x);
    rect.height = (L1.rect.size.height + L1.rect.size.height)/2;
    center.x = (L1.rect.center.x + L2.rect.center. x)/2;
    center.y = (L1.rect.center.y + L2.rect.center.y)/2;
   // circle(img,center,3,Scalar(255,255,255),-1);
    rect.x = center.x - rect.width/2;
    rect.y = center.y - rect.height/2;

//    rect.x = center.x - rect.width/3;
//    rect.y = center.y - rect.height/3;
//    rect.width *= 2.0/3;
//    rect.height *= 2.0/3;
}

void armor::draw_rect(Mat& img)const
{
    rectangle(img,rect,Scalar(255,255,255), -1);

}


void armor::draw_spot(Mat& img)const
{
    circle(img,center,int(rect.height/4),Scalar(0,0,255),-1);
}

int armor::get_target_x(void)
{
    return center.x;
}

int armor::get_target_y(void)
{
    return center.y;
}

int armor::get_average_intensity(const Mat& img)
{
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y <1)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width));
    //imshow("roi",roi);
    average_intensity = mean(roi).val[0];
    return average_intensity;
}

void armor::max_match(vector<LED_Stick>& LED,const int& i, const int&j){
    float f = error_angle + abs(LED.at(i).rect.center.y - LED.at(j).rect.center.y);
    if(!LED.at(i).matched && !LED.at(j).matched)
    {
        LED.at(i).matched = true;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_index = i;
        LED.at(i).match_factor = f;
        LED.at(j).match_factor = f;
    }
    if(LED.at(i).matched && !LED.at(j).matched)
    {
        if(f < LED.at(i).match_factor)
        {
            LED.at(LED.at(i).match_index).matched = false;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
            LED.at(j).matched = true;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;

        }
    }

    if(LED.at(j).matched && !LED.at(i).matched)
    {
        if(f < LED.at(j).match_factor )
        {
            LED.at(LED.at(j).match_index).matched = false;
            LED.at(j).match_factor = f;
            LED.at(j).match_index = i;
            LED.at(i).matched = true;
            LED.at(i).match_factor = f;
            LED.at(i).match_index = j;
        }
    }

    if(LED.at(j).matched && LED.at(i).matched
            && LED.at(i).match_factor > f && LED.at(j).match_factor > f)
    {
        LED.at(LED.at(j).match_index).matched = false;
        LED.at(LED.at(i).match_index).matched = false;
        LED.at(i).matched = true;
        LED.at(i).match_factor = f;
        LED.at(i).match_index = j;
        LED.at(j).matched = true;
        LED.at(j).match_factor = f;
        LED.at(j).match_index = i;
    }
}

bool armor::is_suitable_size(void) const
{
    if(Led_stick[0].rect.size.height * 0.6 < Led_stick[1].rect.size.height
            && Led_stick[0].rect.size.height * 1.4 > Led_stick[1].rect.size.height)
    {
        int h_max = Led_stick[0].rect.size.height > Led_stick[1].rect.size.height?
                    Led_stick[0].rect.size.height : Led_stick[1].rect.size.height;
        if(abs(Led_stick[0].rect.center.y - Led_stick[1].rect.center.y) < 0.5 * h_max)
        {
            if(h_max * 5 > rect.width && h_max < 11*rect.width)
                return true;
        }
    }
    return false;
}

