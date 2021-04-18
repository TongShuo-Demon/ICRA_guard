//
// Created by demon on 2021/3/10.
//
#include "detect_blob.h"





/**
 *　预处理，根据不同的红蓝色设置不同的阈值进行二值化
 * @param  Mat类型图片，颜色
 * @return 　Mat
 * @private
 */
cv::Mat DetectBlob::imagePreProcess(cv::Mat &src,uint8_t enemy_color) {
//    cv::resize(src, src, cv::Size(), 0.5, 0.5);
    std::vector<cv::Mat> channels;       // 通道拆分
    cv::Mat color_channel;
    cv::split(src, channels);               /************************/
    if (enemy_color == 0) {         /*                      */
        color_channel = channels[2];        /* 根据目标颜色进行通道提取 */
    } else if (enemy_color == 1) {    /*        0，红色              */
        color_channel = channels[0];        /************************/
    }

    int light_threshold;
    if(enemy_color == 1){
        light_threshold = 220;
    }else{
        light_threshold = 180;
    }
    cv::threshold(color_channel,color_channel,light_threshold, 255,cv::THRESH_BINARY); // 二值化对应通道

    return color_channel;
}





// 旋转矩形的长宽比,因为程序里面采用拟合椭圆的原因，所以长的就是长的
static double lengthDivWidth(const cv::RotatedRect &rect) {

    return rect.size.height / rect.size.width;
}

// 判断灯条颜色(此函数可以有性能优化).
static uint8_t getBlobColor(const cv::Mat &src, const cv::RotatedRect &blob_position) {
    auto region = blob_position.boundingRect();
    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        return RED;
    } else {
        return BLUE;
    }
}

/**
 *　寻找灯条另外一端的点坐标
 * @param
 * @return 　void
 * @private
 */
void find_newpoint(cv::Point2f start, float angle_rad, float distance, cv::Point2f &pt, int rows, int cols)
{
    pt.x = start.x + distance * cos(angle_rad);
    if (pt.x >= cols)
        pt.x = cols - 1;
    if (pt.x < 0)
        pt.x = 0;

    pt.y = start.y - distance * sin(angle_rad);
    if (pt.y >= rows)
        pt.y = rows - 1;
    if (pt.y < 0)
        pt.y = 0;
}

/**
 *　交换灯条的两个点坐标，保证ｅ1在上面
 * @param
 * @return 　void
 * @private
 */
void check_endpoint(cv::Point2f &e1, cv::Point2f &e2)
{
    if (e1.y > e2.y)
    {
        cv::Point2f temp;
        temp.x = e1.x;
        temp.y = e1.y;

        e1.x = e2.x;
        e1.y = e2.y;

        e2.x = temp.x;
        e2.y = temp.y;
    }
}


//1、描述：
//判断矩形是否在图像内
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：如果在范围内返回true，否则返回flase
bool check_rect(cv::Rect &rect, int rows, int cols)
{
    cv::Rect big_rect(0, 0, cols, rows);
    cv::Rect and_rect;
    and_rect = big_rect & rect;
    if (and_rect.area() < 1)
        return false;

    return true;
}

//1、描述：
//检查待检测值是否在给定范围内，否则更新待值到给定范围内
//2、输入：
//a：待检测值
//hight：范围上限
//low：范围下限
float limit_border(float a, float hight, float low)
{
    if (a > hight)
        return hight;
    if (a < low)
        return low;

    return a;
}

//1、描述：
//首先检查矩形是否在规定范围内
//如果超出范围则去除超出的范围的矩形
//2、输入：
//rect：输入矩形
//rows：行界限
//cols：列界限
//3、输出：
//返回：成功更新矩阵返回true，否则返回false
bool limit_rect(cv::Rect &rect, int rows, int cols)
{
    if (!check_rect(rect, rows, cols))
        return false;
    else
    {
        int x1, y1, x2, y2;
        x1 = limit_border(rect.x, cols - 1, 0);
        y1 = limit_border(rect.y, rows -1 , 0);
        x2 = limit_border(rect.x + rect.width, cols - 1, 0);
        y2 = limit_border(rect.y + rect.height, rows - 1, 0);

        if ((y2 <= y1) || (x2 <= x1))
            return false;

        rect.x = x1;
        rect.y = y1;
        rect.width = x2 - x1;
        rect.height = y2 - y1;

        if ((rect.width < 0) || (rect.height < 0))
            return false;
    }
    return true;
}


//获得两个点之间的距离
float twoPointDistance(cv::Point2f p1, cv::Point2f p2)
{
    float distance = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return distance;
}

//获得两个点连线的水平夹角 horizontal
float getHorizontalAngle(cv::Point2f p1, cv::Point2f p2)
{
    double delta_x, delta_y;
    double rad, angle;
    delta_x = fabsf(p1.x - p2.x);
    delta_y = fabsf(p1.y - p2.y);
    if (delta_x != 0)
    {
        rad = atan(delta_y / delta_x);
        angle = rad * 180 / CV_PI;
    }
    else
        angle = 90;

    return angle;
}


/**
 *　寻找符合条件的灯条
 * @param 原图和腐蚀膨胀后图片
 * @return 　void
 * @private
 */
bool DetectBlob::findBlobsUnderRequirement(cv::Mat &img_backups, cv::Mat &high_light_mask,LightBlobs &blob_list,uint8_t color)
{
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> temp_contours;

    cv::Mat contours_img(img_backups.size() , CV_8UC1, cv::Scalar(0));
    cv::Mat temp_contours_img(img_backups.size() , CV_8UC1, cv::Scalar(0));
    blob_list.clear();  //清除内容

    //只检测最外围轮廓
    cv::findContours(high_light_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

    if (contours.size() > 0) {

        LightBlob temp_blob_list;

        for (int i = 0; i < contours.size(); i++) {

            cv::RotatedRect min_ellipse;
            cv::Rect min_rect;
            cv::Mat min_rect_roi;

            if (contours[i].size() <= 6)         //第i个轮廓的所有像素点数目
                continue;
            else if (contours[i].size() > 6) {

                min_ellipse = fitEllipse(contours[i]);                  //椭圆拟合

                //通过轮廓的长和宽进行限制
                if ((min_ellipse.size.height > SINGLE_BLOB_HEIGHT_MAX_) ||
                    (min_ellipse.size.height < SINGLE_BLOB_HEIGHT_MIN_)) {
#ifdef show_test
                    std::cout << "base筛选灯条：长不符合"<<min_ellipse.size.height<<std::endl;
#endif
                    continue;
                }
                if ((min_ellipse.size.width> SINGLE_BLOB_WIDTH_MAX_) ||
                    (min_ellipse.size.width < SINGLE_BLOB_WIDTH_MIN_)){
#ifdef show_test
                    std::cout << "筛选灯条：宽不符合"<<min_ellipse.size.width<<std::endl;
#endif
                    continue;
                }
                //通过轮廓的旋转角度进行限制
                if ((min_ellipse.angle > SINGLE_BLOB_ROTATE_ANGLE_ ) &&
                    (min_ellipse.angle < (180-SINGLE_BLOB_ROTATE_ANGLE_ ))){
#ifdef show_test
                    std::cout << "筛选灯条：旋转角度不符合"<<min_ellipse.angle<<std::endl;
#endif
                    continue;
                }

                //长和宽的比值进项限制
                double ellipse_h_w_div=lengthDivWidth(min_ellipse);
                if(ellipse_h_w_div > SINGLE_BLOB_H_DIV_W_MAX_ ||
                   ellipse_h_w_div < SINGLE_BLOB_H_DIV_W_MIN_){
#ifdef show_test
                    std::cout << "筛选灯条：长宽比率不符合"<<ellipse_h_w_div<<std::endl;
#endif
                    continue;
                }
                min_rect = min_ellipse.boundingRect();                  //最小外接矩形
                //判断是否超出边界
                if (!limit_rect( min_rect, img_backups.rows, img_backups.cols ))
                {
#ifdef show_test
                    std::cout << "筛选灯条：超出边界"<<std::endl;
#endif
                    continue;
                }

                //对轮廓颜色面积比例进行约束，首先对rgb进行约束，进而对hsv进行约束
                cv::Mat bgr_roi, hsv_roi, color_mask_roi;
                bgr_roi = img_backups(min_rect);
                cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV);
                if (color== BLUE)            //蓝色
                {
                    cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi);
                }
                else     //!因为红色是不连续的
                {
                    cv::Mat hsv1, hsv2;
                    cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1);
                    cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2);
                    color_mask_roi = hsv1 + hsv2;
                }
                int correct_pxl=0;
                for (int j=0; j < color_mask_roi.rows; j++)
                {
                    auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
                    auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

                    for (int k = 0; k < color_mask_roi.cols; k++)
                    {
                        auto hsv_val = hsv_ptr[k];             //第j行，第k列
                        auto b = bgr_ptr[k * 3];
                        auto r = bgr_ptr[k * 3 + 2];

                        if (color == BLUE)    //蓝色
                        {
                            if(hsv_val)
                                correct_pxl++;
                        }
                        else                             //红色
                        {
                            if(hsv_val && r - b > 80)
                                correct_pxl++;
                        }
                    }
                }
                float area_of_contour = cv::contourArea(contours[i]);   //算出面积
                float pixel_persent = (float)correct_pxl / area_of_contour;
                if (pixel_persent < SINGLE_BLOB_LIMIT_PIXEL_)    //0.08
                {
#ifdef show_test
                    cv::putText(contours_img, "no_pxl", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
                    continue;
                }
            }

            temp_contours.emplace_back(contours[i]);

            temp_blob_list.min_ellipse = min_ellipse;            //旋转矩形
            temp_blob_list.min_rect = min_rect;                  //最小外接矩形
            temp_blob_list.blob_length = min_ellipse.size.height; //灯条长度
            temp_blob_list.blob_color=getBlobColor(img_backups,min_ellipse);

            float angle_rad = (90.0 - temp_blob_list.min_ellipse.angle) / 180.0 * CV_PI;
            find_newpoint(temp_blob_list.min_ellipse.center, angle_rad, 0.5 * temp_blob_list.blob_length,
                          temp_blob_list.end_points[0], img_backups.rows, img_backups.cols);

            find_newpoint(temp_blob_list.min_ellipse.center, angle_rad, -0.5 * temp_blob_list.blob_length,
                          temp_blob_list.end_points[1], img_backups.rows, img_backups.cols);
            check_endpoint(temp_blob_list.end_points[0], temp_blob_list.end_points[1]);//如果，交换两个灯条定点坐标

            blob_list.emplace_back(temp_blob_list);
        }

    }
#ifdef show_test
    cv::drawContours(contours_img, contours, -1, cv::Scalar(255), 1, 8);
    cv::drawContours(temp_contours_img, temp_contours, -1, cv::Scalar(255), 1, 8);
    cv::imshow("筛选灯条", temp_contours_img);
    cv::imshow("未筛选灯条", contours_img);
    std::cout<< "blob_list.size:"<<blob_list.size();
#endif
    return  blob_list.size() >= 2;
}


ArmorBox::ArmorBox(const LightBlobs &blobs) :
        light_blobs(blobs){};
/**
 *　匹配装甲板
 * @param
 * @return bool
 * @private
 */
bool DetectBlob::matchArmorBoxes(const cv::Mat &img_backups, const LightBlobs &light_blobs,ArmorBoxes &armor_boxes)
{

    ArmorBoxes().swap(armor_boxes);
//    cv::Mat contours_img(img_backups.size() , CV_8UC3, cv::Scalar(0));
    cv::Mat contours_img = img_backups.clone();
    for (int i = 0; i < light_blobs.size() - 1; i++) {
        for (int j = i + 1; j < light_blobs.size(); j++) {
            float distance, horizontal_angle_diff, rotate_angle_diff, length_ratio;
            length_ratio = light_blobs[i].blob_length / light_blobs[j].blob_length;
            length_ratio = length_ratio < 1 ? length_ratio : 1 / length_ratio;
            //通过两个灯条的长度比率进行判断
            if(length_ratio < DOUBLE_LENGTH_RATION_)
            {
#ifdef show_test
                std::cout << "匹配灯条：长宽比率不符合"<<std::endl;
#endif
                continue;
            }

            //通过两个灯条的距离，距离和较长的灯条比较大小
            distance = twoPointDistance(light_blobs[i].min_ellipse.center, light_blobs[j].min_ellipse.center);
            if (distance < DOUBLE_BLOB_DISTANCE_ || distance >4 * (light_blobs[i].blob_length > light_blobs[j].blob_length
                                                                   ? light_blobs[i].blob_length
                                                                   : light_blobs[j].blob_length)){
#ifdef show_test
                std::cout<< "匹配灯条：灯条的距离不符合"<<distance <<std::endl;
#endif
                continue;
            }
            //通过旋转角度差进行判断
            if ((light_blobs[i].min_ellipse.angle > 90 && light_blobs[j].min_ellipse.angle > 90) ||
                (light_blobs[i].min_ellipse.angle < 90 && light_blobs[j].min_ellipse.angle < 90))
                rotate_angle_diff = fabsf(light_blobs[i].min_ellipse.angle - light_blobs[j].min_ellipse.angle);
            else if (light_blobs[i].min_ellipse.angle < 90 && light_blobs[j].min_ellipse.angle > 90)
                rotate_angle_diff = 180 - light_blobs[j].min_ellipse.angle + light_blobs[i].min_ellipse.angle;
            else if (light_blobs[i].min_ellipse.angle > 90 && light_blobs[j].min_ellipse.angle < 90)
                rotate_angle_diff = 180 - light_blobs[i].min_ellipse.angle + light_blobs[j].min_ellipse.angle;

            if (rotate_angle_diff > DOUBLE_ROTATE_ANGLE_DIFF_) {
#ifdef show_test
                std::cout << "匹配灯条：旋转角度差不符合"<<std::endl;
#endif
                continue;
            }
            //通过与水平角度差进行判断
            horizontal_angle_diff = getHorizontalAngle(light_blobs[i].min_ellipse.center, light_blobs[j].min_ellipse.center);

            if (horizontal_angle_diff > DOUBLE_HORIZONTAL_ANGLE_DIFF_) {
#ifdef show_test
                std::cout << "匹配灯条：水平角度不符合"<<horizontal_angle_diff<<std::endl;
#endif
                continue;
            }
            //判断是否在图像内
            int valid_flag = 0;
            cv::Rect rect1, rect2, rect3;
            rect1 = light_blobs[i].min_rect;
            rect2 = light_blobs[j].min_rect;
            rect3 = rect1 | rect2;
            if (!check_rect(rect3, img_backups.rows, img_backups.cols))
                continue;
            //如果两个灯条是匹配的话，那么valid_flag一定是6
            for (int k = 0; k < light_blobs.size(); k++)
                valid_flag = valid_flag + rect3.contains(light_blobs[k].min_ellipse.center) +
                             rect3.contains(light_blobs[k].end_points[0]) + rect3.contains(light_blobs[k].end_points[1]);
            if (valid_flag != 6)
            {
#ifdef show_test
                std::cout << "匹配灯条：不是6"<<std::endl;
#endif
                continue;
            }
//            if(light_blobs[i].blob_color!=light_blobs[j].blob_color||
//               light_blobs[i].blob_color!=color_type||light_blobs[j].blob_color!=color_type){
//#ifdef show_test
//                std::cout << "匹配灯条：灯条颜色不一致"<<std::endl;
//#endif
//                continue;
//            }

            LightBlobs pair_blobs = {light_blobs.at(i), light_blobs.at(j)};
            //必须添加一个构造函数否则报错
            armor_boxes.emplace_back(pair_blobs);
#ifdef show_test
            //显示结果图
            cv::line(contours_img, light_blobs.at(i).end_points[0], light_blobs.at(i).end_points[1],
                     cv::Scalar(0, 0, 255), 1, 8);
            cv::line(contours_img, light_blobs.at(j).end_points[0], light_blobs.at(j).end_points[1],
                     cv::Scalar(0,0, 255), 1, 8);
            cv::line(contours_img,  light_blobs.at(j).end_points[0], light_blobs.at(i).end_points[0],
                     cv::Scalar(0, 0, 255), 1, 8);
            cv::line(contours_img,  light_blobs.at(j).end_points[1], light_blobs.at(i).end_points[1],
                     cv::Scalar(0, 0, 255), 1, 8);
            cv::imshow("显示匹配灯条",contours_img);
#endif
        }
    }


    for(int m=0;m<armor_boxes.size();m++) {

//        std::cout<<"疑似装甲板数量:"<<pair_blob_armor.size()<<std::endl;
        cv::Point temp[4];
        cv::Point2f e1(armor_boxes[m].light_blobs[0].end_points[0]);
        cv::Point2f e2(armor_boxes[m].light_blobs[0].end_points[1]);
        cv::Point2f p1(armor_boxes[m].light_blobs[1].end_points[0]);
        cv::Point2f p2(armor_boxes[m].light_blobs[1].end_points[1]);

        if (e1.x < p1.x) {
            temp[0] = e1;
            temp[1] = p1;
            temp[2] = p2;
            temp[3] = e2;
        } else {
            temp[0] = p1;
            temp[1] = e1;
            temp[2] = e2;
            temp[3] = p2;
        }
        armor_boxes[m].armor_vertex[0] = temp[0];
        armor_boxes[m].armor_vertex[1] = temp[1];
        armor_boxes[m].armor_vertex[2] = temp[2];
        armor_boxes[m].armor_vertex[3] = temp[3];
    }

    return !armor_boxes.empty();
}


















