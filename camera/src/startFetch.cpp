#include "startFetch.hpp"

using namespace std;

bool DahuaCamera::connectCamera(std::string &filename,
                                preProcessFunction preProcess)
{

    //这一部分为读取配置参数
    getCamParaFromYml(filename, DahuaCamera::daHuaPara);
    if (daHuaPara.IsReadYmlSucc != true)
    {
        std::cerr<<"Failed to get the configur file "<<std::endl;
        return false;
    }

    /* 发现设备 */
    CSystem &systemObj = CSystem::getInstance();//获取实例
    TVector<ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
    if (!isDiscoverySuccess)
    {
        std::cout<<"[error-001]: find device fail."<<endl;
        return false;
    }

    printf(" camera number:%d\n", vCameraPtrList.size());

    if (vCameraPtrList.size() == 0 )
    {
        std::cout<<"[error-002]: no or one devices."<<endl;
        return false;
    }



    /* 连接相机 */
    if (!vCameraPtrList[1]->connect())
    {
        std::cout<<"[error-003]: connect camera failed."<<endl;
        return false;
    } else{
        std::cout<<"connect camera success! "<<endl;
    }

    //获得相机指针0、2
    DahuaCamera::cameraSptr = vCameraPtrList[1];


    /* 按照配置文件进行设置相机参数 */
    int IsSetCamParaSucc = 0;

    camParaConfig(DahuaCamera::cameraSptr, DahuaCamera::daHuaPara, IsSetCamParaSucc);

    if(IsSetCamParaSucc != 0 )
    {
        std::cerr<<"[error-004]: failed to set camera para"<<endl;
        DahuaCamera::cameraSptr->disConnect();
        DahuaCamera::cameraSptr2->disConnect();
        return false;
    }
    else{
        std::cout<<"succeed to set camera para"<<endl;
    }

    /* 创建流对象 */
    IStreamSourcePtr streamPtr = systemObj.createStreamSource(DahuaCamera::cameraSptr);


    if (NULL == streamPtr)
    {
        std::cerr<<"create stream obj  fail."<<endl;
        DahuaCamera::cameraSptr->disConnect();
        DahuaCamera::cameraSptr2->disConnect();
        return false;
    }


    /* 停止抓图 并休眠1ms */
    streamPtr->stopGrabbing();

    usleep(500);
    usleep(500);
    /* 开始取图 */
    bool isStartGrabbingSuccess = streamPtr->startGrabbing();
    if (!isStartGrabbingSuccess)
    {
        std::cerr<<"Start Grabbing fail."<<endl;
        DahuaCamera::cameraSptr->disConnect();
        return false;
    }


    /*创建取流线程*/
    Dahua::Memory::TSharedPtr<StreamRetrieve>  streamThreadSptr(new StreamRetrieve(cameraSptr, //相机指针
                                                                                   daHuaPara,    //大华相机参数
                                                                                   streamPtr,
                                                                                   boost::bind(DahuaCamera::callback,_1),
                                                                                   preProcess));

    if (NULL == streamThreadSptr)
    {
        printf("create thread obj failed.\n");
        streamPtr->stopGrabbing();
        DahuaCamera::cameraSptr->disConnect();
        return false;
    }

    streamThreadSptr->join();             //等待该线程完成

    return true;
}

////不用修改
void DahuaCamera::callback(cv::Mat& img)
{
    if(img.empty() )
    {
        return;
    }
//    cout<<"get image."<<endl;

    static publisher pub("camera_pub",img);
    pub.braodcast(img);

}



//默认的预处理函数,勿动
void DahuaCamera::nothing(cv::Mat &in, cv::Mat &out) {
    out = in;
}