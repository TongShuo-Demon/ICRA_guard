#include "camParaConfig.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
//读取相机参数
void getCamParaFromYml(const std::string &filename, daHuaPara_str &_daHuaPara)
{
    FileStorage fs2(filename, FileStorage::READ);
    if (fs2.isOpened() != true)
    {
        _daHuaPara.IsReadYmlSucc = false;
        std::cerr<<"Failed to get the configur file "<<std::endl;
        return ;
    }

    _daHuaPara.IsGrabModeContinuesMy = (int)fs2["IsGrabModeContinuesMy"];
    _daHuaPara.IsAutoExposureMy = (int)fs2["IsAutoExposureMy"];
    _daHuaPara.frameRateMy  = (int)fs2["frameRateMy"];

    //For Armor
    _daHuaPara.exposeTimeMy_Armor = (int)fs2["exposeTimeMy_Armor"];
    _daHuaPara.gainRawMy_Armor    = (double)fs2["gainRawMy_Armor"];
    _daHuaPara.GammaMy_Armor      = (double)fs2["GammaMy_Armor"];
    //For Rune
    _daHuaPara.exposeTimeMy_Rune = (int)fs2["exposeTimeMy_Rune"];
    _daHuaPara.gainRawMy_Rune    = (double)fs2["gainRawMy_Rune"];
    _daHuaPara.GammaMy_Rune      = (double)fs2["GammaMy_Rune"];

    _daHuaPara.redBalanceRatioMy   = (double)fs2["redBalanceRatioMy"];
    _daHuaPara.greenBalanceRatioMy = (double)fs2["greenBalanceRatioMy"];
    _daHuaPara.blueBalanceRatioMy  = (double)fs2["blueBalanceRatioMy"];

    _daHuaPara.imgWidth = (int)fs2["imgWidth"];
    _daHuaPara.imgHeight = (int)fs2["imgHeight"];

    _daHuaPara.x_offset = (int)fs2["x_off"];
    _daHuaPara.y_offset = (int)fs2["y_off"];


    _daHuaPara.IsReadYmlSucc = true;

    fs2.release();
}
//相机参数设置
void camParaConfig(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara, int &IsSetCamParaSucc)
{
    int success_flag = 0;
    success_flag += setGainRaw(cameraSptr, _daHuaPara.gainRawMy_Armor);
    success_flag += setGamma(cameraSptr, _daHuaPara.GammaMy_Armor);

    success_flag += setGrabMode(cameraSptr, _daHuaPara.IsGrabModeContinuesMy);
    success_flag += setExposureTime(cameraSptr, _daHuaPara.exposeTimeMy_Armor, _daHuaPara.IsAutoExposureMy);

    success_flag += setAcquisitionFrameRate(cameraSptr, _daHuaPara.frameRateMy);
    success_flag += setBalanceRatio(cameraSptr, _daHuaPara.redBalanceRatioMy,
                                    _daHuaPara.greenBalanceRatioMy, _daHuaPara.blueBalanceRatioMy);

    success_flag += setROI(cameraSptr, _daHuaPara.x_offset, _daHuaPara.y_offset,
                           _daHuaPara.imgWidth, _daHuaPara.imgHeight);
    success_flag += setResolution(cameraSptr, _daHuaPara.imgWidth, _daHuaPara.imgHeight);

    IsSetCamParaSucc = success_flag;
}

void ArmorModelParaConfig_runtime(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara)
{
    int success_flag = 0;
    success_flag += setGainRaw(cameraSptr, _daHuaPara.gainRawMy_Armor);
    success_flag += setGamma(cameraSptr, _daHuaPara.GammaMy_Armor);
    success_flag += setExposureTime(cameraSptr, _daHuaPara.exposeTimeMy_Armor, _daHuaPara.IsAutoExposureMy);

    if(!success_flag)
        std::cout<<"camera Parameters set successfully when model change to Armor"<<std::endl;
    else
        std::cerr<<"camera Parameters set successfully when model change to Armor"<<std::endl;
}

void RuneModelParaConfig_runtime(ICameraPtr &cameraSptr, const daHuaPara_str &_daHuaPara)
{
    int success_flag = 0;
    success_flag += setGainRaw(cameraSptr, _daHuaPara.gainRawMy_Rune);
    success_flag += setGamma(cameraSptr, _daHuaPara.GammaMy_Rune);
    success_flag += setExposureTime(cameraSptr, _daHuaPara.exposeTimeMy_Rune, _daHuaPara.IsAutoExposureMy);

    if(!success_flag)
        std::cout<<"camera Parameters set successfully when model change to Rune"<<std::endl;
    else
        std::cerr<<"camera Parameters set successfully when model change to Rune"<<std::endl;
}