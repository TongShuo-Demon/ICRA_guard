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
#include <thread>
#include <iostream>
#define PORT 7000
#define QUEUE 1//连接请求队列
int conn;
void thread_task()
{
}

//radar对robot进行消息发送的信息
struct carinfo {
//        char name[20];     // 注意：CPU访问内存的时候，对访问地址有对齐的要求，一般为2的幂次方。所以，有的数据被填充，以满足对齐要求。
    int ID;
    float x;
    float y;
};

carinfo ros_zuobiao;

int main()
{
    //printf("%d\n",AF_INET);//IPv4协议
    printf("%d\n",SOCK_STREAM);//字节流套接字
    int ss = socket(AF_INET, SOCK_STREAM, 0);//若成功则返回一个sockfd（套接字描述符）
    //printf("%d\n",ss);
    struct sockaddr_in server_sockaddr;//一般是储存地址和端口的。用于信息的显示及存储使用
    /*设置 sockaddr_in 结构体中相关参数*/
    server_sockaddr.sin_family = AF_INET;
    server_sockaddr.sin_port = htons(PORT);//将一个无符号短整型数值转换为网络字节序，即大端模式(big-endian)　
    //printf("%d\n",INADDR_ANY);
    //INADDR_ANY就是指定地址为0.0.0.0的地址，这个地址事实上表示不确定地址，或“所有地址”、“任意地址”。
    //一般来说，在各个系统中均定义成为0值。
    server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);//将主机的无符号长整形数转换成网络字节顺序。　
    if(bind(ss, (struct sockaddr* ) &server_sockaddr, sizeof(server_sockaddr))==-1)
    {
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

    char buffer[1024];
    //创建另外一个线程
    //std::thread t(thread_task);
    //t.join();
    //char buf[1024];
    //主线程
    while(1)
    {
        //这里把send注释掉了，所以这个程序中server只能是接收client端的数据并能给client发送数据，即使不注释掉也没用，因为没有对是否有数据传入和传入
        //进行判断所以按照下面的代码这样写，每次都要先让server输入后才能输出client传过来的数据，若是server不输入则程序无法向下走就没有client发送过来的输出，
        //而且每次显示也只能是一行，这样显示就全是错的了，所以就需要select和FD_ISSET的判断了
        // memset(buf, 0 ,sizeof(buf));
        // if(fgets(buf, sizeof(buf),stdin) != NULL) {
        // send(conn, buf, sizeof(buf), 0);
        // }

        memset(buffer, 0 ,sizeof(buffer));
        int len = recv(conn, buffer, sizeof(buffer), 0);//从TCP连接的另一端接收数据。
        /*该函数的第一个参数指定接收端套接字描述符；
        第二个参数指明一个缓冲区，该缓冲区用来存放recv函数接收到的数据；
        第三个参数指明buf的长度；
        第四个参数一般置0*/
        if(strcmp(buffer, "exit\n") == 0)//如果没有收到TCP另一端发来的数据则跳出循环不输出
        {
            break;
        }
        
        //必须要有返回数据， 这样才算一个完整的请求
       // send(conn, buffer, len , 0);//向TCP连接的另一端发送数据。
//    send(conn, buffer, len , 0);//向TCP连接的另一端发送数据。
    memcpy(&ros_zuobiao,buffer,sizeof(carinfo));  //解析过程，将字符串的内容写入到p2所在内存完成解析
   // printf("x: %f , y: %f,  id: %i\n", ros_zuobiao.x,ros_zuobiao.y,ros_zuobiao.ID);//如果有收到数据则输出数据
   std::cout << "x:" << ros_zuobiao.x << "y: " << ros_zuobiao.y << "z: " << std::endl;
}
    close(conn);//因为accpet函数连接成功后还会生成一个新的套接字描述符，结束后也需要关闭
    close(ss);//关闭socket套接字描述符
    return 0;
}
