//
// Created by demon on 2021/4/15.
//

#include "init.h"

namespace guard_init
{
    float yolov5_conf_thres ;
    float yolov5_iou_thres ;

    cv::Mat cv_camera_matrix;
    cv::Mat cv_dist_coeffs;
    cv::Mat cv_Rtag2camera;
    cv::Mat cv_Ttag2camera;


    std::string yolov5_detect_path;
    std::string mlp_position_path;
    std::string yolov5_name_path;
    std::string backgroud_path;
    std::string video_source_path_right;
    torch::DeviceType device_type;


    cv::Point3f tag_pts[4];               //标签坐标
    cv::Point2f pixel_pts[4];             // 像素坐标


    float limit_fuse;




    void loadModel(bool is_gpu);
    void yolov5Init(bool is_gpu);


}

void guard_init::loadModel(bool is_gpu) {

    if (torch::cuda::is_available() && is_gpu) {
        device_type = torch::kCUDA;
        std::cout << "using cuda" << std::endl;
    } else {
        device_type = torch::kCPU;
    }


    cv::FileStorage file_1("../cfg/guard_config.yml", cv::FileStorage::READ);
    if(file_1.isOpened())
    {
        std::cout << "try to read armor camera configuration parameter" << std::endl;

        file_1["yolov5_model_path"] >> guard_init::yolov5_detect_path;
        file_1["position_mlp_path"] >> guard_init::mlp_position_path;
        file_1["yolov5_name_path"] >> guard_init::yolov5_name_path;
        file_1["backgroud_path"] >> guard_init::backgroud_path;
        file_1["video_source_path_right"] >> guard_init::video_source_path_right;
        file_1["yolov5_conf_thres"] >> guard_init::yolov5_conf_thres;
        file_1["yolov5_iou_thres"] >> guard_init::yolov5_iou_thres;
        pixel_pts[0].x = file_1["tag_image_x1"];
        pixel_pts[0].y = file_1["tag_image_y1"];
        pixel_pts[1].x = file_1["tag_image_x2"];
        pixel_pts[1].y = file_1["tag_image_y2"];
        pixel_pts[2].x = file_1["tag_image_x3"];
        pixel_pts[2].y = file_1["tag_image_y3"];
        pixel_pts[3].x = file_1["tag_image_x4"];
        pixel_pts[3].y = file_1["tag_image_y4"];
        limit_fuse = file_1["limit_fuse"];

        file_1.release();
    }
    else
    {
        std::cerr << "fail to read guard configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read guard configuration parameter" << std::endl;

    cv::FileStorage file_2("../cfg/guard_camera_config.yml", cv::FileStorage::READ);
    if (file_2.isOpened())
    {
        std::cout << "try to read armor camera configuration parameter" << std::endl;

        file_2["cameraMatrix"] >> guard_init::cv_camera_matrix;
        file_2["distCoeffs"] >> guard_init::cv_dist_coeffs;
        file_2["Rtag2world"] >> guard_init::cv_Rtag2camera;
        file_2["Ttag2world"] >> guard_init::cv_Ttag2camera;

        file_2.release();
    }else
    {
        std::cerr << "fail to read camera configuration parameter" << std::endl;
        return;
    }
    std::cout << "succed to read camera configuration parameter" << std::endl;

    guard_init::tag_pts[0] = cv::Point3f(-75, -75, 0);
    guard_init::tag_pts[1] = cv::Point3f(75, -75, 0);
    guard_init::tag_pts[2] = cv::Point3f(75, 75, 0);
    guard_init::tag_pts[3] = cv::Point3f(-75, 75, 0);

}


void guard_init::yolov5Init(YOLOv5Detector detector, std::vector<std::string> &class_names){

    //! load class names from dataset for visualization
    class_names = detector.loadNames(guard_init::yolov5_name_path);
    if (class_names.empty()) {
        return ;
    }
    cv::Mat img(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    //! run once to warm up
    std::cout << "Run once on empty image" << std::endl;
    auto temp_img = cv::Mat::zeros(img.rows, img.cols, CV_32F);
    detector.run(img, 1.0f, 1.0f);
}