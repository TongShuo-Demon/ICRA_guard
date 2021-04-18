//
// Created by demon on 2021/3/12.
//

#ifndef LIBTORCH_YOLOV5_CLIENT_H
#define LIBTORCH_YOLOV5_CLIENT_H

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include "iostream"
#include <Eigen/Core>
#define ROBOT_PORT 8000
#define ROBOT_BUFFER_SIZE 1024

/*
 * 实现的功能是client到server的半双工通信，server只能接受接收client发送过来的消息，但是不能向client发送消息。
 * 参考网址1：https://blog.csdn.net/weixin_43795921/article/details/85307133
 * 参考网址2：https://zhuanlan.zhihu.com/p/80373327
 * 学习地址：http://c.biancheng.net/view/2131.html
 */


//radar对robot进行消息发送的信息
struct carinfo {
//        char name[20];     // 注意：CPU访问内存的时候，对访问地址有对齐的要求，一般为2的幂次方。所以，有的数据被填充，以满足对齐要求。
    int ID;
    float x;
    float y;
};


class Radar2RobotSocket{

public:
    Radar2RobotSocket();
    ~Radar2RobotSocket();
    ///帧头aa，是否存在目标，车ID，x，y，pose，三种状态（0:自旋，1：左右摇摆，3：前进）dd，一共8个字节
    bool sendBuf(carinfo &car_info);    //Eigen::Vector3f P_world
    bool isConnect();




private:
    ///定义socket_client_
    int socket_client_;
    ///定义server_address_
    struct sockaddr_in server_address_;
};



#define QUEUE 20//连接请求队列

struct robot2radar{

    float x;
    float y;
    float z;
    float w;

};

class Robot2RadarSocket{

    public:
        Robot2RadarSocket();
        bool  isConnect();
        bool receiveBuf(  robot2radar  &ros_zuobiao);  //, robot2radar  &ros_zuobiao
        ~Robot2RadarSocket();

    private:
        int conn;
        int ss;
        struct sockaddr_in server_sockaddr;//一般是储存地址和端口的。用于信息的显示及存储使用
};











#endif //LIBTORCH_YOLOV5_CLIENT_H
