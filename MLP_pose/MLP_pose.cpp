//
// Created by demon on 2021/3/17.
//

#include "MLP_pose.h"



PositionPredict::PositionPredict(const std::string& model_path, const torch::DeviceType& device_type): device_(device_type) {

    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module_ = torch::jit::load(model_path);
    }
    catch (const c10::Error& e) {
        std::cerr << "Error loading the model!\n";
        std::exit(EXIT_FAILURE);
    }

    half_ = (device_ != torch::kCPU);
    module_.to(device_);

    if (half_) {
        module_.to(torch::kHalf);
    }

    module_.eval();

}
cv::Point2f PositionPredict:: camera_mlp(const cv::Rect bbox ){

    std::vector<float>  input;
    input.push_back(bbox.tl().x/1280.);
    input.push_back(bbox.tl().y/1024.);
    input.push_back(bbox.br().x/1280.);
    input.push_back(bbox.br().y/1024.);


    auto tensor_bbox = torch::from_blob(input.data(), { 1, 4}).to(device_); //数据变为张亮


    if (half_) {
        tensor_bbox = tensor_bbox.to(torch::kHalf);   //改为16位浮点型
    }

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(tensor_bbox);
    // inference
    torch::jit::IValue output = module_.forward(inputs);
    auto out = output.toTensor();
    auto out2 = out.toType(torch::kFloat);

    cv::Point2f p_world;
    cv::Mat o_Mat(cv::Size(2, 1), CV_32FC1, out.data_ptr());

    float x = o_Mat.ptr<float>(0)[0];
    float y = o_Mat.ptr<float>(0)[1];
    p_world.x = x*8.08;
    p_world.y = y*4.48;
//    std::cout <<"output: "  << p_world << "  " << out << "Mat:" << o_Mat<< std::endl;
    return p_world;
}


