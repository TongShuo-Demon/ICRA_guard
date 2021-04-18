#include <iostream>
#include <memory>
#include <chrono>
#include "detect_yolo.h"
#include <vector>
#include "detect_tag.h"
#include "tag_and_camera.h"
#include "client.h"
#include "MLP_pose.cpp"
#include "switch_function.h"
#include <shm.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "init.h"
#include "small_tool.h"


cv::VideoCapture capvideo; //创建VideoCapture对象




void main_task(YOLOv5Detector detector,cv::Mat src_image, std::vector<std::string> class_names,PositionPredict mlp_pose_camera,
               TagandCamera tagandcamera,    cv::Mat ros,
               Eigen::Matrix3f R_cam_tag,Eigen::Vector3f T_cam_tag){
    //! inference
    auto result = detector.run(src_image, guard_init::yolov5_conf_thres, guard_init::yolov5_iou_thres);

    Eigen::Vector3f math_world;
    carinfo car_information;
    Point tmp;


    for (int i = 0; i < result.size(); i++) {

        if (result[i].class_idx == 0) {                           //判断是否是车
            car_information.x = 0;
            car_information.y = 0;
            car_information.ID = 0;

            //!数学结算的方法
            Point2f temp_center = getCenterPoint(result[i].bbox);
            math_world = tagandcamera.pixel2World(temp_center, R_cam_tag, T_cam_tag);
            if(math_world[0] > 4.48) math_world[0] = 4.48;
            if(math_world[1] > 8.08) math_world[1] = 8.08;
            //!神经网络的方法
            Point2f MLP_world = mlp_pose_camera.camera_mlp(result[i].bbox);
            //!进行目标融合
            Point2f math_temp;
            math_temp.x = 4.48 - math_world[0];
            math_temp.y = 8.08 - math_world[1];
            if(getDistance(math_temp,MLP_world )  > guard_init::limit_fuse){
                car_information.x = math_temp.x;
                car_information.y = math_temp.y;
                tmp.x = 448 - math_world[0] * 100;
                tmp.y = 808 - math_world[1] * 100;
            }else{
                tmp.x = MLP_world.y*100;
                tmp.y = MLP_world.x*100;
                car_information.x = MLP_world.x ;
                car_information.y = MLP_world.y ;
            }


            for (int j = 0; j < result.size(); j++) {

#ifdef OBJECT_SHOW
                std::stringstream ss;
                std::string object_label;
                auto font_face = cv::FONT_HERSHEY_DUPLEX;
                auto font_scale = 1.0;
                int thickness = 1;
                int baseline=0;
                Size label_size ;
                std::string label_world;
#endif

                switch (result[j].class_idx) {
                    case 1 :
#ifdef OBJECT_SHOW
                        ss << std::fixed << std::setprecision(2) << result[j].score;
                        object_label = class_names[1] + " " + ss.str();
                        label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
                        cv::rectangle(src_image, result[j].bbox, cv::Scalar(0, 0, 255), 2);
#endif
                        if (isInside(result[j].bbox, result[i].bbox)) {
                            car_information.ID = 1;                 //red1
#ifdef OBJECT_SHOW
                            cv::rectangle(src_image, result[i].bbox, cv::Scalar(0, 0, 255), 2);
                            circle(ros, tmp, 10, Scalar(0, 0, 255),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
                            cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;

                            cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
                                        cv::Point(10, 20),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );

                            cv::rectangle(src_image, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
                                          cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
                                          cv::Scalar(0, 0, 255), -1);
                            cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                        font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
                            label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
                            cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
                                        FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
#endif
                        }

                        break;
                    case 2 :
#ifdef OBJECT_SHOW
                        ss << std::fixed << std::setprecision(2) << result[j].score;
                        object_label = class_names[2] + " " + ss.str();
                        label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
                        cv::rectangle(src_image, result[j].bbox, cv::Scalar(0, 0, 255), 2);
#endif
                        if (isInside(result[j].bbox, result[i].bbox)) {
                            car_information.ID = 2;                //red2
#ifdef OBJECT_SHOW
                            cv::rectangle(src_image, result[i].bbox, cv::Scalar(0, 0, 255), 2);
                            circle(ros, tmp, 10, Scalar(0, 0, 255),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
                            cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;

                            cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
                                        cv::Point(10, 30),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );

                            cv::rectangle(src_image,
                                          cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
                                          cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
                                          cv::Scalar(0, 0, 255), -1);
                            cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                        font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
                            label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
                            cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
                                        FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
#endif
                        }

                        break;

                    case 3 :
#ifdef OBJECT_SHOW
                        ss << std::fixed << std::setprecision(2) << result[j].score;
                        object_label = class_names[3] + " " + ss.str();
                        label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
                        cv::rectangle(src_image, result[j].bbox, cv::Scalar(255, 0, 0), 2);
#endif
                        if (isInside(result[j].bbox, result[i].bbox)) {
                            car_information.ID = 3;                //blue1
#ifdef OBJECT_SHOW
                            cv::rectangle(src_image, result[i].bbox, cv::Scalar(255, 0, 0), 2);
                            circle(ros, tmp, 10, Scalar(255, 0, 0),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
                            cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
                            cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
                                        cv::Point(10, 40),FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 0, 0), 1 );
                            cv::rectangle(src_image,
                                          cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
                                          cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
                                          cv::Scalar(255, 0, 0), -1);
                            cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                        font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
                            label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
                            cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
                                        FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
#endif
                        }

                        break;
                    case 4 :
#ifdef OBJECT_SHOW
                        ss << std::fixed << std::setprecision(2) << result[j].score;
                        object_label = class_names[4] + " " + ss.str();
                        label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
                        cv::rectangle(src_image, result[j].bbox, cv::Scalar(255, 0, 0), 2);
#endif
                        if (isInside(result[j].bbox, result[i].bbox)) {
                            car_information.ID = 4;                //blue2
#ifdef OBJECT_SHOW
                            cv::rectangle(src_image, result[i].bbox, cv::Scalar(255, 0, 0), 2);
                            circle(ros, tmp, 10, Scalar(255, 0, 0),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
                            cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
                            cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
                                        cv::Point(10, 50),FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 0, 0), 1 );
                            cv::rectangle(src_image,
                                          cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
                                          cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
                                          cv::Scalar(255, 0, 0), -1);
                            cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
                                        font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
                            label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
                            cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
                                        FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
#endif
                        }
                        break;
                }

            }

        }
    }


}



int main()
{
    guard_init::loadModel(true);
    /****************************************前期的初始化*******************************************************/
    //! load network
    YOLOv5Detector detector(guard_init::yolov5_detect_path, guard_init::device_type);
    std::vector<std::string> class_names;
    guard_init::yolov5Init(detector,class_names);

#ifdef VIDEO
    capvideo.open(guard_init::video_source_path_right);
    if (!capvideo.isOpened()) {   //检查是否能正常打开视频文件
        std::cout << "fail to open Base video" << std::endl;
    }
#elif  defined(CAMERA)
    subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同
#ifdef VIDEO_WRITE
        int video_cnt=0;
        std::string videodirname = "../video/"+now_iso_str+"/";
        const char * videodirnamestr = videodirname.c_str();
        mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
        std::string video_dart_name = videodirname+"realtime.avi";

        SaveVideo.open(video_dart_name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 20, VideoSize);

        while(!SaveVideo.isOpened()){
            video_cnt++;
            SaveVideo.open(video_dart_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, VideoSize);
            if(video_cnt>=10){
                break;
            }
        }
        video_cnt=0;
#endif
#endif

/***********************相机姿态确定(MLP与数学解算)**************************************************************/
    Eigen::Matrix3f R_cam_tag;    //使坐标点从世界坐标系旋转到相机坐标系
    Eigen::Vector3f T_cam_tag;
    TagandCamera tagandcamera;
    PositionPredict mlp_pose_camera(guard_init::mlp_position_path,torch::kCPU);
    tagandcamera.tag2Camera(R_cam_tag, T_cam_tag);
    cv::Mat src_image,ros;
#ifdef socket
    while(!radar_robot_socket.isConnect()){
        cout << "socket 没有连接成功"  << endl;
    }
#endif

while(1){


    auto start = std::chrono::high_resolution_clock::now();
    ros = imread(guard_init::backgroud_path);
#ifdef VIDEO
    capvideo >> src_image;

    if (src_image.empty())//如果某帧为空则退出循环
        break;
#elif  defined(CAMERA)
    camera_sub.get(src_image);
            if (src_image.empty())//如果某帧为空则退出循环
                break;
#ifdef VIDEO_WRITE
            g_cnt_Of_dart++;
            if(g_cnt_Of_dart%FRAME_DARTS_FREQ == 0)
            {
                g_cnt_Of_dart = 0;
                SaveVideo << src_image;
            }
#endif
#endif

    main_task(detector,src_image,class_names,mlp_pose_camera,tagandcamera,
              ros,R_cam_tag,T_cam_tag);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // It should be known that it takes longer time at first time
            std::cout << "total takes : " << duration.count() << " ms" << std::endl;
            cv::putText(src_image, "FPS: " + to_string(int(1.0/duration.count()*1000)), cv::Point(10, 10),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );


    imshow("ros", ros);
    imshow("src_image", src_image);
    waitKey(1);

    }


}




//
//
//
//
//
//
//int main() {
//
//    guard_init::loadModel(true);
//    Radar2RobotSocket  radar_robot_socket;
//
//    boost::posix_time::ptime time_now(boost::posix_time::second_clock::local_time());
//    std::string now_iso_str(to_iso_string(time_now));
//    cv::Mat src_image;
//
//#ifdef VIDEO_WRITE
//    cv::Size VideoSize(1280,1024);
//    cv::VideoWriter SaveVideo;                 //类
//    int FRAME_DARTS_FREQ = 1;                 //常量
//    int g_cnt_Of_dart = 0;                   //全局变量
//#endif
//
///****************************************前期的初始化*******************************************************/
//    //! load network
//    YOLOv5Detector detector(guard_init::yolov5_detect_path, guard_init::device_type);
//    std::vector<std::string> class_names;
//    guard_init::yolov5Init(detector,class_names);
//
//#ifdef VIDEO
//    capvideo.open(guard_init::video_source_path_right);
//    if (!capvideo.isOpened()) {   //检查是否能正常打开视频文件
//        std::cout << "fail to open Base video" << std::endl;
//    }
//#elif  defined(CAMERA)
//    subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同
//#ifdef VIDEO_WRITE
//        int video_cnt=0;
//        std::string videodirname = "../video/"+now_iso_str+"/";
//        const char * videodirnamestr = videodirname.c_str();
//        mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
//        std::string video_dart_name = videodirname+"realtime.avi";
//
//        SaveVideo.open(video_dart_name, cv::VideoWriter::fourcc('M', 'P', '4', '2'), 20, VideoSize);
//
//        while(!SaveVideo.isOpened()){
//            video_cnt++;
//            SaveVideo.open(video_dart_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, VideoSize);
//            if(video_cnt>=10){
//                break;
//            }
//        }
//        video_cnt=0;
//#endif
//#endif
//
//
///***********************相机姿态确定(MLP与数学解算)**************************************************************/
//    Eigen::Matrix3f R_cam_tag;    //使坐标点从世界坐标系旋转到相机坐标系
//    Eigen::Vector3f T_cam_tag;
//    TagandCamera tagandcamera;
//    PositionPredict mlp_pose_camera(guard_init::mlp_position_path,torch::kCPU);
//    tagandcamera.tag2Camera(R_cam_tag, T_cam_tag);
//
//
//#ifdef socket
//    while(!radar_robot_socket.isConnect()){
//        cout << "socket 没有连接成功"  << endl;
//    }
//#endif
//
//
//
//    while (1) {
//
//        cv::Mat ros = imread(guard_init::backgroud_path);
//
//        auto start = std::chrono::high_resolution_clock::now();
//#ifdef VIDEO
//        capvideo >> src_image;
//
//        if (src_image.empty())//如果某帧为空则退出循环
//            break;
//#elif  defined(CAMERA)
//        camera_sub.get(src_image);
//            if (src_image.empty())//如果某帧为空则退出循环
//                break;
//#ifdef VIDEO_WRITE
//            g_cnt_Of_dart++;
//            if(g_cnt_Of_dart%FRAME_DARTS_FREQ == 0)
//            {
//                g_cnt_Of_dart = 0;
//                SaveVideo << src_image;
//            }
//#endif
//#endif
//
//        //! inference
//        auto result = detector.run(src_image, guard_init::yolov5_conf_thres, guard_init::yolov5_iou_thres);
//
//        Eigen::Vector3f math_world;
//        carinfo car_information;
//        Point tmp;
//
//
//        for (int i = 0; i < result.size(); i++) {
//
//            if (result[i].class_idx == 0) {                           //判断是否是车
//                car_information.x = 0;
//                car_information.y = 0;
//                car_information.ID = 0;
//
//                //!数学结算的方法
//                Point2f temp_center = getCenterPoint(result[i].bbox);
//                math_world = tagandcamera.pixel2World(temp_center, R_cam_tag, T_cam_tag);
//                if(math_world[0] > 4.48) math_world[0] = 4.48;
//                if(math_world[1] > 8.08) math_world[1] = 8.08;
//                //!神经网络的方法
//                Point2f MLP_world = mlp_pose_camera.camera_mlp(result[i].bbox);
//                //!进行目标融合
//                Point2f math_temp;
//                math_temp.x = 4.48 - math_world[0];
//                math_temp.y = 8.08 - math_world[1];
//                if(getDistance(math_temp,MLP_world )  > guard_init::limit_fuse){
//                    car_information.x = math_temp.x;
//                    car_information.y = math_temp.y;
//                    tmp.x = 448 - math_world[0] * 100;
//                    tmp.y = 808 - math_world[1] * 100;
//                }else{
//                    tmp.x = MLP_world.y*100;
//                    tmp.y = MLP_world.x*100;
//                    car_information.x = MLP_world.x ;
//                    car_information.y = MLP_world.y ;
//                }
//
//
//                for (int j = 0; j < result.size(); j++) {
//
//#ifdef OBJECT_SHOW
//                    std::stringstream ss;
//                    std::string object_label;
//                    auto font_face = cv::FONT_HERSHEY_DUPLEX;
//                    auto font_scale = 1.0;
//                    int thickness = 1;
//                    int baseline=0;
//                    Size label_size ;
//                    std::string label_world;
//#endif
//
//                    switch (result[j].class_idx) {
//                        case 1 :
//#ifdef OBJECT_SHOW
//                            ss << std::fixed << std::setprecision(2) << result[j].score;
//                            object_label = class_names[1] + " " + ss.str();
//                            label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
//                            cv::rectangle(src_image, result[j].bbox, cv::Scalar(0, 0, 255), 2);
//#endif
//                            if (isInside(result[j].bbox, result[i].bbox)) {
//                                car_information.ID = 1;                 //red1
//#ifdef OBJECT_SHOW
//                                cv::rectangle(src_image, result[i].bbox, cv::Scalar(0, 0, 255), 2);
//                                circle(ros, tmp, 10, Scalar(0, 0, 255),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
//                                cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
//
//                                cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
//                                            cv::Point(10, 20),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//
//                                cv::rectangle(src_image, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
//                                              cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
//                                              cv::Scalar(0, 0, 255), -1);
//                                cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
//                                            font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
//                                label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
//                                cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
//                                            FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
//#endif
//                            }
//
//                            break;
//                        case 2 :
//#ifdef OBJECT_SHOW
//                            ss << std::fixed << std::setprecision(2) << result[j].score;
//                            object_label = class_names[2] + " " + ss.str();
//                            label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
//                            cv::rectangle(src_image, result[j].bbox, cv::Scalar(0, 0, 255), 2);
//#endif
//                            if (isInside(result[j].bbox, result[i].bbox)) {
//                                car_information.ID = 2;                //red2
//#ifdef OBJECT_SHOW
//                                cv::rectangle(src_image, result[i].bbox, cv::Scalar(0, 0, 255), 2);
//                                circle(ros, tmp, 10, Scalar(0, 0, 255),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
//                                cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
//
//                                cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
//                                            cv::Point(10, 30),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//
//                                cv::rectangle(src_image,
//                                              cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
//                                              cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
//                                              cv::Scalar(0, 0, 255), -1);
//                                cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
//                                            font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
//                                label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
//                                cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
//                                            FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
//#endif
//                            }
//
//                            break;
//
//                        case 3 :
//#ifdef OBJECT_SHOW
//                            ss << std::fixed << std::setprecision(2) << result[j].score;
//                            object_label = class_names[3] + " " + ss.str();
//                            label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
//                            cv::rectangle(src_image, result[j].bbox, cv::Scalar(255, 0, 0), 2);
//#endif
//                            if (isInside(result[j].bbox, result[i].bbox)) {
//                                car_information.ID = 3;                //blue1
//#ifdef OBJECT_SHOW
//                                cv::rectangle(src_image, result[i].bbox, cv::Scalar(255, 0, 0), 2);
//                                circle(ros, tmp, 10, Scalar(255, 0, 0),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
//                                cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
//                                cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
//                                            cv::Point(10, 40),FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 0, 0), 1 );
//                                cv::rectangle(src_image,
//                                              cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
//                                              cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
//                                              cv::Scalar(255, 0, 0), -1);
//                                cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
//                                            font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
//                                label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
//                                cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
//                                            FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
//#endif
//                            }
//
//                                break;
//                                case 4 :
//#ifdef OBJECT_SHOW
//                                    ss << std::fixed << std::setprecision(2) << result[j].score;
//                                    object_label = class_names[4] + " " + ss.str();
//                                    label_size = cv::getTextSize(object_label, font_face, font_scale, thickness, &baseline);
//                                    cv::rectangle(src_image, result[j].bbox, cv::Scalar(255, 0, 0), 2);
//#endif
//                                    if (isInside(result[j].bbox, result[i].bbox)) {
//                                        car_information.ID = 4;                //blue2
//#ifdef OBJECT_SHOW
//                                        cv::rectangle(src_image, result[i].bbox, cv::Scalar(255, 0, 0), 2);
//                                        circle(ros, tmp, 10, Scalar(255, 0, 0),-1,8); //第三个参数表示点的半径，第四个参数选择颜色。这样子我们就画出了绿色的空心点
//                                        cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
//                                        cv::putText(src_image, "info:" + to_string(car_information.ID) +" " + to_string(car_information.x)+" " + to_string(car_information.y) ,
//                                                    cv::Point(10, 50),FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 0, 0), 1 );
//                                        cv::rectangle(src_image,
//                                                      cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - label_size.height - 5),
//                                                      cv::Point(result[i].bbox.tl().x + label_size.width, result[i].bbox.tl().y),
//                                                      cv::Scalar(255, 0, 0), -1);
//                                        cv::putText(src_image, object_label, cv::Point(result[i].bbox.tl().x, result[i].bbox.tl().y - 5),
//                                                    font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
//                                        label_world =  "x: " + to_string(car_information.x) + ", y:" +to_string(car_information.y);
//                                        cv::putText(src_image, label_world, cv::Point(result[i].bbox.tl().x-10, result[i].bbox.tl().y -30),
//                                                    FONT_HERSHEY_PLAIN , 1, cv::Scalar(255, 255, 255), 1);
//#endif
//                                    }
//                                break;
//                            }
//
//                    }
//
//                }
//            }
//
//
//#ifdef socket
//            if(radar_robot_socket.sendBuf(car_information)){
//                cout << "发送成功: " << car_information.ID << " : "  << car_information.x <<  " : "  << car_information.y << endl;
//                cv::putText(src_image, "socket:ID " + to_string(car_information.ID), cv::Point(10, 20),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//                cv::putText(src_image, "socket:x  " + to_string(car_information.x), cv::Point(10, 30),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//                cv::putText(src_image, "socket:y " + to_string(car_information.y), cv::Point(10, 40),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//            }
//#endif
//
//            auto end = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//            // It should be known that it takes longer time at first time
//            std::cout << "total takes : " << duration.count() << " ms" << std::endl;
//
//
//            cv::putText(src_image, "FPS: " + to_string(int(1.0/duration.count()*1000)), cv::Point(10, 10),FONT_HERSHEY_PLAIN , 1, cv::Scalar(0, 0, 255), 1 );
//
//            imshow("ros", ros);
//            imshow("src_image", src_image);
//            waitKey(1);
//
//        }
//
//        cv::destroyAllWindows();
//        return 0;
//}
//
//
//
//
