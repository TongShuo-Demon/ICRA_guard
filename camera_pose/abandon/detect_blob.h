//
// Created by demon on 2021/3/10.
//

#ifndef LIBTORCH_YOLOV5_DETECT_BLOB_H
#define LIBTORCH_YOLOV5_DETECT_BLOB_H


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#define BLUE 1
#define RED  0

#define show_test

/******************* 灯条类定义 ***********************/
class LightBlob {
public:
    double blob_length;     //灯条长度
    uint8_t blob_color;     //灯条颜色

    cv::RotatedRect min_ellipse;      //拟合后的旋转矩形
    cv::Rect min_rect;               //最小外接矩形
    cv::Point2f end_points[2];      //灯条端点坐标


//    LightBlob(cv::RotatedRect &r,  uint8_t color) : min_ellipse(r),blob_color(color) {
//        blob_length=min_ellipse.size.height;
//    };
    LightBlob() = default;

    };

typedef std::vector<LightBlob> LightBlobs;


/******************* 装甲板类定义　**********************/
class ArmorBox{
public:

    LightBlobs light_blobs;         //多个灯条
    cv::Point2f armor_vertex[4];      //装甲板四个定点坐标
    cv::Rect2d rect;               //装甲板区域

    int ArmorID;              //装甲板数字

    explicit ArmorBox(const LightBlobs &blobs=LightBlobs());

};
typedef std::vector<ArmorBox> ArmorBoxes;





class DetectBlob{


    public:

        LightBlobs lightblobs;
        ArmorBoxes armorboxes;

        bool findBlobsUnderRequirement(cv::Mat &img_backups, cv::Mat &high_light_mask,LightBlobs &blob_list,uint8_t color);
        bool matchArmorBoxes(const cv::Mat &img_backups, const LightBlobs &light_blobs,ArmorBoxes &armor_boxes);
        cv::Mat imagePreProcess(cv::Mat &src,uint8_t enemy_color);
    private:

//!灯条筛选的参数
    int SINGLE_BLOB_HEIGHT_MIN_ = 1;
    int SINGLE_BLOB_HEIGHT_MAX_ = 300;

    int SINGLE_BLOB_WIDTH_MIN_ = 1;
    int SINGLE_BLOB_WIDTH_MAX_ = 300;
    int SINGLE_BLOB_ROTATE_ANGLE_ = 45;

    float SINGLE_BLOB_H_DIV_W_MIN_ = 1;
    float SINGLE_BLOB_H_DIV_W_MAX_ = 15;
    float SINGLE_BLOB_LIMIT_PIXEL_=0.08;

//!匹配灯条筛选的参数
    float DOUBLE_LENGTH_RATION_=0.5;
    float DOUBLE_BLOB_DISTANCE_=0.6;
    float DOUBLE_ROTATE_ANGLE_DIFF_=25;
    float DOUBLE_HORIZONTAL_ANGLE_DIFF_=45;


};

















#endif //LIBTORCH_YOLOV5_DETECT_BLOB_H
