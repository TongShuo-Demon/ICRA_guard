#include "shm.hpp"


    publisher::publisher(const char* node_name,cv::Mat& img):
            img_h(img.rows),img_w(img.cols),img_c(img.channels()),shm_name(string(node_name))
    {

        publisher::lock_name = string(node_name)+"_lock";
        publisher::data_images = string(node_name)+"_image";
        publisher::data_message = string(node_name)+"_message";

        bool res = boost::interprocess::shared_memory_object::remove(publisher::shm_name.c_str()); //首先检查内存是否被释放
        boost::interprocess::named_mutex::remove(publisher::lock_name.c_str());

        //托管共享内存
        publisher::managed_shm = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,
                                                                publisher::shm_name.c_str(),
                                                               img.total()*img.elemSize()+12+1024);//1228800

        publisher::named_mtx = new boost::interprocess::named_mutex(boost::interprocess::create_only, publisher::lock_name.c_str());

        publisher::shm_image = publisher::managed_shm->construct<char>(publisher::data_images.c_str())[img.total()*img.elemSize()](0);

        publisher::shm_image_message = publisher::managed_shm->construct<int>(publisher::data_message.c_str())[3](0);

    }

    void publisher::braodcast(cv::Mat& img) {

        publisher::named_mtx->lock();

        memcpy(publisher::shm_image, img.data, img.total()*img.elemSize());

        *publisher::shm_image_message = publisher::img_w;
        *(publisher::shm_image_message+1) = publisher::img_h;
        *(publisher::shm_image_message+2) = img.type();

        publisher::named_mtx->unlock();
    }

    publisher::~publisher() {
        boost::interprocess::shared_memory_object::remove(publisher::shm_name.c_str());
        boost::interprocess::named_mutex::remove(publisher::lock_name.c_str());
    }
//订阅器
    subscriber::subscriber(const char* node_name)
    {
        subscriber::lock_name = string(node_name)+"_lock";
        subscriber::data_images=string(node_name)+"_image";
        subscriber::data_message=string(node_name)+"_message";

        subscriber::managed_shm = new boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only
                ,string(node_name).c_str());

        subscriber::named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::open_only,
                subscriber::lock_name.c_str());

        subscriber::shm_image = subscriber::managed_shm->find<char>(subscriber::data_images.c_str());
        subscriber::shm_image_message = subscriber::managed_shm->find<int>(subscriber::data_message.c_str());
    }

    void subscriber::get(cv::Mat &img)
    {
        subscriber::named_mtx->lock();

        img = cv::Mat(cv::Size(*subscriber::shm_image_message.first,*(subscriber::shm_image_message.first+1)),
                          *(subscriber::shm_image_message.first+2));

        memcpy(img.data, subscriber::shm_image.first, img.total()*img.elemSize());

        subscriber::named_mtx->unlock();
    }
