//
// Created by demon on 2021/3/21.
//

#ifndef POSITION_PREDICT_PY_MLP_POSE_H
#define POSITION_PREDICT_PY_MLP_POSE_H

#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

class PositionPredict{

public:

    PositionPredict(const std::string& model_path, const torch::DeviceType& device_type);
    cv::Point2f camera_mlp(const cv::Rect bbox );

private:
    torch::jit::script::Module module_;
    torch::Device device_;
    bool half_ =   false;
};














#endif //POSITION_PREDICT_PY_MLP_POSE_H
