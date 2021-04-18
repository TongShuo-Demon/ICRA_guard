//
// Created by demon on 2021/3/12.
//

#include "client.h"


Radar2RobotSocket :: Radar2RobotSocket(){
    ///创建一个socket套接字,af为地址族（Address Family），AF_INET 表示 IPv4 地址,
    /// type 为数据传输方式/套接字类型,protocol 表示传输协议，可以将 protocol 的值设为 0，系统会自动推演出应该使用什么协议
    socket_client_ = socket(AF_INET,SOCK_STREAM, 0);
    ///设置socket的各种属性
    memset(&server_address_, 0, sizeof(server_address_));
    server_address_.sin_family = AF_INET;
    server_address_.sin_port = htons(ROBOT_PORT); //服务器端口
    server_address_.sin_addr.s_addr = inet_addr("192.168.1.179"); //服务器ip，inet_addr用于IPv4的IP转换（十进制转换为二进制）,这里填写服务器的ip地址

}
bool Radar2RobotSocket :: isConnect(){

    //连接服务器，成功返回0，错误返回-1
    if (connect(socket_client_, (struct sockaddr *)&server_address_, sizeof(server_address_)) < 0)
    {
        perror("connect");
        exit(1);
        return false;
    }else{
        return true;
    }
}




bool Radar2RobotSocket :: sendBuf(carinfo &car_info){


    char sendbuf[12];

//    carinfo car;

//    car.x= P_world[0];
//    car.y = P_world[1];
//    car.ID = 1;

    memset(sendbuf,0,sizeof(sendbuf));         // 对该内存段进行清
    memcpy(sendbuf,&car_info,sizeof(carinfo));     // 把这个结构体中的信息从内存中读入到字符串temp中


    int is_succes = send(socket_client_, sendbuf, sizeof(sendbuf) ,0); ///发送strlen(send_)

    if(is_succes) {
        memset(sendbuf, 0, sizeof(sendbuf));//接受或者发送完毕后把数组中的数据全部清空（置0）
        return true;
    }

    return false;

}


Radar2RobotSocket ::~Radar2RobotSocket(){

    close(socket_client_);   ///关闭套接字
}








//**********************以下是采集数据进行接收使用******************************************

#define PORT 7000

Robot2RadarSocket ::  Robot2RadarSocket(){

    //printf("%d\n",AF_INET);//IPv4协议
    printf("%d\n",SOCK_STREAM);//字节流套接字
    ss = socket(AF_INET, SOCK_STREAM, 0);//若成功则返回一个sockfd（套接字描述符）
    //printf("%d\n",ss);
//    struct sockaddr_in server_sockaddr;//一般是储存地址和端口的。用于信息的显示及存储使用
    /*设置 sockaddr_in 结构体中相关参数*/
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(PORT);//将一个无符号短整型数值转换为网络字节序，即大端模式(big-endian)　
    //printf("%d\n",INADDR_ANY);
    //INADDR_ANY就是指定地址为0.0.0.0的地址，这个地址事实上表示不确定地址，或“所有地址”、“任意地址”。
    //一般来说，在各个系统中均定义成为0值。
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);//将主机的无符号长整形数转换成网络字节顺序。　
}

bool Robot2RadarSocket :: isConnect()
{
    if(bind(ss, (struct sockaddr* ) &server_sockaddr, sizeof(server_sockaddr))==-1)
    {
        std::cout << "not connet" << std::endl;
        perror("bind");
        exit(1);
    }
    if(listen(ss, QUEUE) == -1)
    {
        perror("listen");
        exit(1);
    }

    struct sockaddr_in client_addr;
    socklen_t length = sizeof(client_addr);
    ///成功返回非负描述字，出错返回-1
    conn = accept(ss, (struct sockaddr*)&client_addr, &length);
    //如果accpet成功，那么其返回值是由内核自动生成的一个全新描述符，代表与所返回客户的TCP连接。
    //accpet之后就会用新的套接字conn
    if( conn < 0 )
    {
        perror("connect");
        exit(1);
    }
    return true;
}



bool Robot2RadarSocket :: receiveBuf(  robot2radar  &ros_zuobiao){

    char buffer[1024];


    memset(buffer, 0 ,sizeof(buffer));
    recv(conn, buffer, sizeof(buffer), 0);//从TCP连接的另一端接收数据。
//    printf("%s", buffer);//如果有收到数据则输出数据
    //必须要有返回数据， 这样才算一个完整的请求
//    send(conn, buffer, len , 0);//向TCP连接的另一端发送数据。
    memcpy(&ros_zuobiao,buffer,sizeof(robot2radar));  //解析过程，将字符串的内容写入到p2所在内存完成解析

    std::cout <<"the number of byte: " <<sizeof(robot2radar) <<   "x:" << ros_zuobiao.x  << "  y:" << ros_zuobiao.y << "  z:" << ros_zuobiao.z << "  w:" << ros_zuobiao.w <<std::endl;
}



Robot2RadarSocket ::  ~Robot2RadarSocket(){

    close(conn);//因为accpet函数连接成功后还会生成一个新的套接字描述符，结束后也需要关闭
    close(ss);//关闭socket套接字描述符

}