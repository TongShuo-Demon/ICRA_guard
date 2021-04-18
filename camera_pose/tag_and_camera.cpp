//
// Created by demon on 2021/3/9.
//


#include "tag_and_camera.h"



//!畸变矫正分为两种，分别是图像整体矫正和单个点矫正
//!去畸变
cv::Point2f TagandCamera :: pixelDistortionCorrection(cv::Point2f &connor)
{
    Eigen::Vector3f pixel;
    Eigen::Vector3f P_cam, P_distortion;  //3行1列，distortion畸变

    Eigen::Matrix3f cameraMatrix;
    Eigen::Matrix<float, 5, 1> distCoeffs;
    cv2eigen(camMatrix_,cameraMatrix);
    cv2eigen(distCoeff_,distCoeffs);


    pixel << connor.x, connor.y, 1;  //像素坐标
    P_cam = cameraMatrix.inverse() * pixel; //inverse是进行逆变换，求出Xc/Zc,即归一化后的坐标,内参矩阵

    float r2 = pow(P_cam.x(), 2) + pow(P_cam.y(), 2);//pow是求x的y次方
    //公式求出畸变坐标点
    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * (r2 + 2 * pow(P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + distCoeffs[3] * ( r2 + 2 * pow( P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();

    P_distortion.z() = 1;

    cv::Point2f P;  //矫正后的点

    P.x=cameraMatrix(0,0)*P_distortion.x()+cameraMatrix(0,2);
    P.y=cameraMatrix(1,1)*P_distortion.y()+cameraMatrix(1,2);

    return P;
}





TagandCamera :: TagandCamera()
{

    camMatrix_ = guard_init::cv_camera_matrix;
    distCoeff_ = guard_init::cv_dist_coeffs;
    Rtag2camera_ =  guard_init::cv_Rtag2camera;
    Ttag2camera_ =  guard_init::cv_Ttag2camera;
}


//!这一步获得相机与标签的信息，也就是得到相机的外参
//!mouse control
bool TagandCamera :: tag2Camera(Eigen::Matrix3f &R, Eigen::Vector3f &T)
{
    world_.clear();
    image_.clear();
    world_.push_back( guard_init::tag_pts[0]);
    world_.push_back( guard_init::tag_pts[1]);
    world_.push_back( guard_init::tag_pts[2]);
    world_.push_back( guard_init::tag_pts[3]);

    image_.emplace_back(guard_init::pixel_pts[0]);
    image_.emplace_back(guard_init::pixel_pts[1]);
    image_.emplace_back(guard_init::pixel_pts[2]);
    image_.emplace_back(guard_init::pixel_pts[3]);

    cv::Mat rvec, tvec;
    Mat rotM = Mat::eye(3,3, CV_32F);
    Mat rotT = Mat::eye(3,3, CV_32F);
    cv::solvePnP(world_, image_, camMatrix_, distCoeff_,rvec,tvec,false);
    Rodrigues(rvec, rotM);  //将旋转向量变换成旋转矩阵

    cout << rotM << endl << tvec <<  endl;

    cv2eigen(rotM,R);
    cv2eigen(tvec,T);

    return true;
}




//!换算到tag坐标下面
Eigen::Vector3f TagandCamera :: pixel2World( cv::Point2f connor,Eigen::Matrix3f rv, Eigen::Vector3f tv)
{
    Eigen::Vector3f pixel;
    Point2f tmp = pixelDistortionCorrection(connor);
    pixel << tmp.x,tmp.y,1.0;

    Eigen::Matrix3f cameraMatrix;
    cv2eigen(camMatrix_,cameraMatrix);


    Eigen::Matrix3f R_tag_cam,R_world_tag,rv2;
    Eigen::Matrix<float, 3, 1> T_world_tag,T_tag_cam,tv2;

    cv2eigen(Rtag2camera_,R_world_tag);
    cv2eigen(Ttag2camera_,T_world_tag);

    tv2 << tv[0],tv[1],tv[2];
    R_tag_cam = rv.transpose();
    T_tag_cam = -rv.transpose() * tv2;

    rv = R_world_tag * R_tag_cam;
    tv2 = R_world_tag * T_tag_cam + T_world_tag;

    rv2 = rv.transpose();
    tv2 = -rv.transpose() * tv2;

    //计算 invR * T
    Eigen::Matrix3f invR;
    Eigen::Vector3f transPlaneToCam,worldPtCam,worldPtPlane,scale_worldPtPlane,world;
    invR=rv2.inverse();
    transPlaneToCam =  invR * tv2 ;
    //[x,y,z] = invK * [u,v,1]
    worldPtCam = cameraMatrix.inverse()*pixel;
    //[x,y,1] * invR
    worldPtPlane = invR * worldPtCam;
    //zc,存在疑问（因为考虑到车是二维平面）
    float scale = transPlaneToCam[2] / worldPtPlane[2];
    //zc * [x,y,1] * invR
    scale_worldPtPlane = scale * worldPtPlane;
    //[X,Y,Z]=zc*[x,y,1]*invR - invR*T
    world = scale_worldPtPlane - transPlaneToCam;

    world << (world[0])/1000,(world[1])/1000,1;
    return world;
}



