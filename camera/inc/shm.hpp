#ifndef DELTAIPS_SHM_HPP
#define DELTAIPS_SHM_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <string.h>

using namespace std;
//用于相机图像共享内存的发布
class publisher{
public:
    publisher(const char* node_name,cv::Mat& img);
    ~publisher();

    void braodcast(cv::Mat& img);

private:

    char* shm_image;
    int* shm_image_message;

    std::string shm_name;
    std::string lock_name;
    std::string data_images;
    std::string data_message;

    int img_w,img_h,img_c;

    boost::interprocess::managed_shared_memory* managed_shm;   //托管共享内存
    boost::interprocess::named_mutex* named_mtx;              //声明互斥对象
};

//用于相机图像共享内存的订阅
class subscriber{

public:
    subscriber(const char* node_name);

    void get(cv::Mat& img);

private:

    std::pair<char*,size_t > shm_image;
    std::pair<int*,size_t > shm_image_message;

    std::string shm_name;
    std::string lock_name;
    std::string data_images;
    std::string data_message;

    boost::interprocess::managed_shared_memory* managed_shm;          //托管共享内存
    boost::interprocess::named_mutex* named_mtx;
};

/*------------------------------发布器-模板类--------------------*/
template <typename T1>          //用于下位机数据反馈共享内存创建
class shm_publisher_create{
public:
    shm_publisher_create(const char* node_name, const int RECEIVE_DATA_LENGTH)
            :shm_name(string(node_name)), //创建的名字
             per_data_bytes(sizeof(T1)),  //位数
             data_length(RECEIVE_DATA_LENGTH) //接收数据长度
    {
        lock_name = string(node_name)+"_lock"; //定义互斥变量的名称　//这里的命名规则与发布器对应
        data_name = string(node_name)+"_data"; //定义数据的名称
        update_name = string(node_name)+"_update";//更新标志位

        boost::interprocess::shared_memory_object::remove(shm_name.c_str()); //首先检查内存是否被释放
        boost::interprocess::named_mutex::remove(lock_name.c_str()); //检查互斥变量是否被释放

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,
                                                                     shm_name.c_str(),
                                                                     per_data_bytes*data_length + 4 + 1024);
        // 互斥变量
        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::create_only,
                lock_name.c_str());
        // 变量
        user_data = managed_shm->construct<T1>(data_name.c_str())[data_length](0);
        update_flag = managed_shm->construct<int>(update_name.c_str())[1](0);
    }

    ~shm_publisher_create() {
        boost::interprocess::shared_memory_object::remove(shm_name.c_str());
        boost::interprocess::named_mutex::remove(lock_name.c_str());
        managed_shm->destroy<T1>(data_name.c_str());
        managed_shm->destroy<int>(update_name.c_str());
    }

    void broadcast(T1* data)
    {
        named_mtx->lock();

        memcpy(user_data, data, per_data_bytes*data_length);
        *update_flag = 1;

        named_mtx->unlock();
    }

private:
    T1* user_data;
    int* update_flag;
    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;

    int per_data_bytes,data_length;
};

/*---------------------------------------------------*/
template <typename T1>              //用于下位机数据反馈共享内存打开
class shm_publisher_open{
public:
    shm_publisher_open(const char* node_name) //这里传入的node_name必须是已经创建的
            :shm_name(string(node_name)),
             per_data_bytes(sizeof(T1))
    {
        lock_name = string(node_name) + "_lock"; //这里的命名规则与发布器对应
        data_name = string(node_name) + "_data";
        update_name = string(node_name)+"_update";

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, //注意open_only
                shm_name.c_str());

        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::open_only,//注意open_only
                lock_name.c_str());

        user_data = managed_shm->find<T1>(data_name.c_str());
        update_flag = managed_shm->find<int>(update_name.c_str());
    }

    // 返回值表示当前取得值是否更新
    bool broadcast(T1* data)
    {
        named_mtx->lock();

        memcpy(user_data.first, data, user_data.second*per_data_bytes);
        update_flag.first[0] = 1;

        named_mtx->unlock();
    }
    ~shm_publisher_open() {
        boost::interprocess::shared_memory_object::remove(shm_name.c_str());
        boost::interprocess::named_mutex::remove(lock_name.c_str());
        managed_shm->destroy<T1>(data_name.c_str());
        managed_shm->destroy<int>(update_name.c_str());
    }

private:
    std::pair<T1*,size_t > user_data;
    std::pair<int*,size_t > update_flag;

    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;
    uint8_t per_data_bytes;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;
};

/*-------------------订阅器--------------------------------*/
template <typename T2>                  //用于下位机数据反馈共享内存订阅
class shm_subscriber{
public:
    shm_subscriber(const char* node_name) //这里传入的node_name必须是已经创建的
            :shm_name(string(node_name)),
             per_data_bytes(sizeof(T2))
    {
        lock_name = string(node_name) + "_lock"; //这里的命名规则与发布器对应
        data_name = string(node_name) + "_data";
        update_name = string(node_name)+"_update";

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, //注意open_only
                shm_name.c_str());

        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::open_only,//注意open_only
                lock_name.c_str());

        user_data = managed_shm->find<T2>(data_name.c_str());
        update_flag = managed_shm->find<int>(update_name.c_str());
    }

    // 返回值表示当前取得值是否更新
    bool get(T2* data)
    {
        named_mtx->lock();
        if(update_flag.first[0]==1) //检查标志位是否置1
        {
            update_flag.first[0]=0;
            memcpy(data, user_data.first,user_data.second*per_data_bytes);
            named_mtx->unlock();
            return true;
        } else {
            named_mtx->unlock();
            return false;
        }
//        update_flag.first[0]=0;
//        memcpy(data, user_data.first,user_data.second*per_data_bytes);
//        named_mtx->unlock();
    }

private:
    std::pair<T2*,size_t > user_data;
    std::pair<int*,size_t > update_flag;

    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;
    uint8_t per_data_bytes;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;
};

#endif //DELTAIPS_SHM_HPP
