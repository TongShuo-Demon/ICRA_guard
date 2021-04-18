//
// Created by demon on 2021/3/29.
//

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<stdio.h>
#include <string>
#include <opencv2/highgui/highgui_c.h>
#include <vector>
#include "iostream"
using namespace cv;
using namespace std;
Mat src;
Mat dst;
vector<Point> P;
int n=0;
void on_mouse(int event, int x, int y, int flags, void* ustc)
{
    static Point pre_pt ;
    char temp_1[20];


    if (event == CV_EVENT_LBUTTONDOWN)
    {
        n++;
        dst.copyTo(src);
        pre_pt = Point(x, y);
        P.emplace_back(pre_pt);
        sprintf(temp_1,"x:%d,y:%d",x,y);
        putText(src,temp_1,Point(x,y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(150,55,245));
        imshow("src", src);
        src.copyTo(dst);//确保画线操作是在src上进行
    }

    if(n>=4)
    {
//        imshow("org",org);
        cvDestroyAllWindows();
    }
}





int main()
{
    //注意：这一步必须要有，不然进行不了鼠标操作
    namedWindow("src", WINDOW_AUTOSIZE);//WINDOW_AUTOSIZE:系统默认,显示自适应
    src = imread("/media/demon/仝硕/ICRA/tmp4/0.bmp", 1);//1:为原图颜色,0:为灰度图，黑白颜色
    src.copyTo(dst);

    setMouseCallback("src", on_mouse);
    imshow("src", src);
    waitKey(0);
    for(int i=0; i < P.size(); i++ ){

        std::cout <<"P" <<  P[i] << std::endl;
    }

    return 0;
}

