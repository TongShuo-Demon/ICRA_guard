#include <iostream>
#include <memory>
#include <chrono>
#include "detect_yolo.h"
#include <vector>
#include "detect_tag.h"
#include "client.h"
#include <shm.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>



#define MyColor 1



std::vector<std::string> LoadNames(const std::string& path) {
    // load class names
    std::vector<std::string> class_names;
    std::ifstream infile(path);
    if (infile.is_open()) {
        std::string line;
        while (getline (infile,line)) {
            class_names.emplace_back(line);
        }
        infile.close();
    }
    else {
        std::cerr << "Error loading the class names!\n";
    }

    return class_names;
}


void Demo(cv::Mat& img,
          const std::vector<Detection>& detections,
          const std::vector<std::string>& class_names,
          bool label = true) {

    for (const auto& detection : detections) {
        const auto& box = detection.bbox;
        float score = detection.score;
        int class_idx = detection.class_idx;




        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);



        if (label) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << score;
            std::string s = class_names[class_idx] + " " + ss.str();


            auto font_face = cv::FONT_HERSHEY_DUPLEX;
            auto font_scale = 1.0;
            int thickness = 1;
            int baseline=0;
            auto s_size = cv::getTextSize(s, font_face, font_scale, thickness, &baseline);
            cv::rectangle(img,
                          cv::Point(box.tl().x, box.tl().y - s_size.height - 5),
                          cv::Point(box.tl().x + s_size.width, box.tl().y),
                          cv::Scalar(0, 0, 255), -1);
            cv::putText(img, s, cv::Point(box.tl().x, box.tl().y - 5),
                        font_face , font_scale, cv::Scalar(255, 255, 255), thickness);


        }
    }

    cv::namedWindow("Result", cv::WINDOW_AUTOSIZE);
    cv::imshow("Result", img);
    cv::waitKey(5);
}



cv::Size VideoSize(1280,1024);

cv::VideoWriter SaveVideo;            //类

int FRAME_DARTS_FREQ = 1;                 //常量

int g_cnt_Of_dart = 0;                   //全局变量



Robot2RadarSocket  radar_robot_socket;

std::ofstream FileOut;




bool isInside(Rect rect1, Rect rect2)

{
    return (rect1 == (rect1&rect2));

}





#define angle
//#define mlp

int main() {

    subscriber camera_sub("camera_pub");  //定义共享内存，共享内存名称需要与预设的相同



    boost::posix_time::ptime time_now(boost::posix_time::second_clock::local_time());
    std::string now_iso_str(to_iso_string(time_now));

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


    cv::Mat src_image;
/****************************************前期的初始化*******************************************************/
    // check if gpu flag is set
    bool is_gpu = true;
    // set device type - CPU/GPU
    torch::DeviceType device_type;
    if (torch::cuda::is_available() && is_gpu) {
        device_type = torch::kCUDA;
        std::cout << "__cuda__"  << std::endl;
    } else {
        device_type = torch::kCPU;
    }
    //! load class names from dataset for visualization
    std::vector<std::string> class_names = LoadNames("../yolov5/weights/my.names");
    if (class_names.empty()) {
        return -1;
    }
    cv::Mat img(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    // load network
    std::string weights = "/home/demon/CLionProjects/ICRA_guard/exp53/weights/best.torchscript.pt";
    YOLOv5Detector detector(weights, device_type);
    // run once to warm up
    std::cout << "Run once on empty image" << std::endl;
    auto temp_img = cv::Mat::zeros(img.rows, img.cols, CV_32F);
    detector.run(img, 1.0f, 1.0f);
    // set up threshold
    float conf_thres = 0.4;
    float iou_thres = 0.5;


    while(!radar_robot_socket.isConnect()){
        cout << "socket 没有连接成功"  << endl;
    }
    int number = 0;

    robot2radar  ros_zuobiao;
    int number_caiji = 0;
    while(1) {

        auto start = std::chrono::high_resolution_clock::now();

        camera_sub.get(src_image);
        if (src_image.empty())//如果某帧为空则退出循环
            break;
        imshow("dd",src_image);

        g_cnt_Of_dart++;
        if(g_cnt_Of_dart%FRAME_DARTS_FREQ == 0)
        {
            g_cnt_Of_dart = 0;
            SaveVideo << src_image;
        }

        //! inference
        auto result = detector.run(src_image, conf_thres, iou_thres);

        char key_value = waitKey(30);
        number_caiji++;         //key_value == 'w' number_caiji == 10
        if( number_caiji == 5) {
            number_caiji = 0;
            cout << "start catch" << endl;
            number++;
            for (int i = 0; i < result.size(); i++) {
                if (result[i].class_idx == 0) {
                    radar_robot_socket.receiveBuf(ros_zuobiao);

                    for (int j = 0; j < result.size(); j++) {
                        if (result[j].class_idx == MyColor && isInside(result[j].bbox, result[i].bbox)) {
                            //!创建日志文件
                            std::string LogName = "../date_weizhi/4-01-01/" + now_iso_str + "_" + to_string(number) + ".txt";
                            std::string bmpName = "../date_weizhi/4-01-01/" + now_iso_str + "_" + to_string(number) + ".bmp";
                            const char *lognamestr = LogName.c_str();
                            FileOut.open(lognamestr);
                            //!输出系统时间
                            FileOut << "\n**********************************************" << std::endl;
                            FileOut << now_iso_str << ": save infomation " << std::endl;
                            FileOut << "zuobiao:" << ros_zuobiao.x << " " << ros_zuobiao.y << " " << ros_zuobiao.z
                                    << " " << ros_zuobiao.w << std::endl;
                            FileOut << "infomation: " << result[i].bbox.tl().x << " " << result[i].bbox.tl().y << " "
                                    << result[i].bbox.br().x << " " << result[i].bbox.br().y << std::endl;
                            cv::imwrite(bmpName, src_image(result[i].bbox));

                        }

                    }
                }

            }
            FileOut.close();

        }

        // visualize detections  opt["view-img"].as<bool>()
        if (true) {
            Demo(src_image, result, class_names);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // It should be known that it takes longer time at first time
        std::cout << "total takes : " << duration.count() << " ms" << std::endl;


    }


    cv::destroyAllWindows();
    return 0;
}











