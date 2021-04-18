//
// Created by demon on 2021/3/9.
//

#ifndef LIBTORCH_YOLOV5_TAG_AND_CAMERA_H
#define LIBTORCH_YOLOV5_TAG_AND_CAMERA_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "init.h"

using namespace cv;
using namespace std;

class TagandCamera{

    private:

        vector<Point3f> world_;
        vector<Point2f> image_;
        cv::Mat camMatrix_,distCoeff_;
        cv::Mat Rtag2camera_,Ttag2camera_;


    public:
        TagandCamera();

        bool tag2Camera(Eigen::Matrix3f &R, Eigen::Vector3f &T);
        Eigen::Vector3f pixel2World( cv::Point2f connor,Eigen::Matrix3f rv, Eigen::Vector3f tv);

        cv::Point2f pixelDistortionCorrection(cv::Point2f &connor);
};















#endif //LIBTORCH_YOLOV5_TAG_AND_CAMERA_H
