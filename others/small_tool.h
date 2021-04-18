//
// Created by demon on 2021/4/16.
//

#ifndef POSITION_PREDICT_PY_SMALL_TOOL_H
#define POSITION_PREDICT_PY_SMALL_TOOL_H
#include "iostream"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;


double getDistance (Point2f point0,Point2f point1 );
Point getCenterPoint(Rect rect);
bool isInside(Rect rect1, Rect rect2);













#endif //POSITION_PREDICT_PY_SMALL_TOOL_H
