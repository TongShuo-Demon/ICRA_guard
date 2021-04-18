//
// Created by demon on 2021/3/20.
//



#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;


#define div 10

#define WIDTH (8080/div)
#define HEIGHT (4480/div)

#define width (1000/div)
#define height (200/div)



#define line_width 1
#define  line_framework  10




int main(){


    cv::Mat ros = cv::Mat::zeros(HEIGHT, WIDTH,CV_8UC1);
    cv::Mat out(ros.size(), ros.type(), cv::Scalar(255));//全白图


    cv::Rect rect1(0, (1000/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)  B1
    rectangle(out,rect1,Scalar(0, 0, 0),line_width, LINE_8,0);
    cv::Mat subImage1 = out(rect1);
    subImage1.setTo(0);


    cv::Rect rect2((1500/div), (2140/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)   B2
    rectangle(out,rect2,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage2 = out(rect2);
    subImage2.setTo(0);


    cv::Rect rect3((3540/div), (935/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)   B4
    rectangle(out,rect3,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage3 = out(rect3);
    subImage3.setTo(0);

    cv::Rect rect4((3540/div), (3345/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)  B6
    rectangle(out,rect4,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage4 = out(rect4);
    subImage4.setTo(0);

    cv::Rect rect5((5780/div), (2140/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)   B8
    rectangle(out,rect5,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage5 = out(rect5);
    subImage5.setTo(0);

    cv::Rect rect6((7080/div), (3280/div), width, height);//左上坐标（x,y）和矩形的长(x)宽(y)   B9
    rectangle(out,rect6,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage6 = out(rect6);
    subImage6.setTo(0);

    cv::Rect rect7((1500/div), (3480/div), height, width);//左上坐标（x,y）和矩形的长(x)宽(y)   B3
    rectangle(out,rect7,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage7 = out(rect7);
    subImage7.setTo(0);

    cv::Rect rect8((6380/div), 0, height, width);//左上坐标（x,y）和矩形的长(x)宽(y)   B7
    rectangle(out,rect8,Scalar(0),line_width, LINE_8,0);
    cv::Mat subImage8 = out(rect8);
    subImage8.setTo(0);

    cv::Point2f P1 = Point2f(80.8*5,41.3*5);
    cv::Point2f P2 = Point2f(77.3*5,44.8*5);
    cv::Point2f P3 = Point2f(80.8*5,48.3*5);
    cv::Point2f P4 = Point2f(84.3*5,44.8*5);

    line(out,P1,P2,Scalar(0),line_width, LINE_8,0);
    line(out,P1,P4,Scalar(0),line_width, LINE_8,0);
    line(out,P2,P3,Scalar(0),line_width, LINE_8,0);
    line(out,P3,P4,Scalar(0),line_width, LINE_8,0);
    std::vector<cv::Point> contour;
    contour.push_back(P1);
    contour.push_back(P2);
    contour.push_back(P3);
    contour.push_back(P4);

    vector<vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours(out, contours, -1, cv::Scalar::all(0), cv::FILLED);


    line(out,Point(0,0),Point(0,HEIGHT),Scalar(0),line_framework, LINE_8,0);
    line(out,Point(0,0),Point(WIDTH,0),Scalar(0),line_framework, LINE_8,0);
    line(out,Point(0,HEIGHT),Point(WIDTH,HEIGHT),Scalar(0),line_framework, LINE_8,0);
    line(out,Point(WIDTH,0),Point(WIDTH,HEIGHT),Scalar(0),line_framework, LINE_8,0);


    cv::imwrite("ros.png",out);

    cv::imshow("ddd",out);
    cv::waitKey(0);




}

