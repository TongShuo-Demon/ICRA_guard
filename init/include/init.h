//
// Created by demon on 2021/4/15.
//

#ifndef POSITION_PREDICT_PY_INIT_H
#define POSITION_PREDICT_PY_INIT_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect_yolo.h"


namespace guard_init
{
    extern cv::Mat cv_camera_matrix;
    extern cv::Mat cv_dist_coeffs;
    extern cv::Mat cv_Rtag2camera;
    extern cv::Mat cv_Ttag2camera;
    extern float yolov5_conf_thres ;
    extern float yolov5_iou_thres ;
    extern std::string yolov5_detect_path;
    extern std::string mlp_position_path;
    extern std::string yolov5_name_path;
    extern std::string backgroud_path;
    extern std::string video_source_path_right;
    extern torch::DeviceType device_type;

    extern cv::Point3f tag_pts[4];               //标签坐标
    extern cv::Point2f pixel_pts[4];             // 像素坐标


    extern float limit_fuse;

    void loadModel(bool is_gpu) ;
    void yolov5Init(YOLOv5Detector detector, std::vector<std::string> &class_names);
}

#endif //POSITION_PREDICT_PY_INIT_H
