//
// Created by demon on 2021/4/16.
//

#include "small_tool.h"



//!计算欧式距离
//!输入为两个点
double getDistance(Point2f point0,Point2f point1 )
{
    double distance;

    distance = powf((point0.x - point1.x),2) + powf((point0.y - point1.y),2);

    distance = sqrtf(distance);

    return distance;
}



//!判断rect是否在两一个rect内部
bool isInside(Rect rect1, Rect rect2)
{
    return (rect1 == (rect1&rect2));

}



//!获取矩形中心点的功能
Point getCenterPoint(Rect rect)

{
    Point cpt;

    cpt.x = rect.x + cvRound(rect.width/2.0);

    cpt.y = rect.y + cvRound(rect.height/2.0);

    return cpt;

}



