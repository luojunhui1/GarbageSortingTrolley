//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_DETECT_H
#define GARBAGESORTINGTROLLEY_DETECT_H

#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/persistence.hpp>

#include "defs.h"
#include "data.h"
#include "log.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;

class Detector
{
private:
    Mat gray[4];//存储对应四个颜色的二值化图像
    Mat gray_binary;//二值化图像

    //神经网络输入输出结构
    Mat input_blob;
    Net net;
    std::vector<String> out_names;
    std::vector<Mat> outs;

    Point class_id_point;

    vector<Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;
    vector<int> indices;

    Mat angle_map;//像素-角度映射表
    Mat distance_map;//像素-距离映射表

    double x, y, z;//目标点相机坐标系下的x,y,z坐标
    double confidence;//目标置信度

    queue<int> target_type_queue;//历史目标队列
    vector<int> target_type_array;//历史目标出现次数向量

    //针对国赛策略增加数据结构，存储场地上对应位置的垃圾种类
    int ground_target_type[10];
    bool initialized_ground_target_type;
    vector<int> left_ground_target_type;
    vector<int> right_ground_target_type;

    Mat yellow_region;

public:
    bool is_find_target;//是否找到目标
    int target_type;//目标种类

    int is_picked_up;//是否成功抓取到了目标，
    int is_get_clamp_position;//是否可以执行抓取动作

    float distance;//目标距离
    float angle;//目标角度
    double target_confidence;//目标置信度

    Rect target_box;//目标矩形框

    int last_target_type;//上次抓取垃圾的目标种类

    bool is_get_putback_position;//是否达到了指定垃圾区前方
    int direction;//运动方向

    Mat close_area_image;
private:
    static const int distance_length = 7;//距离滑动平均滤波器处理数据长度

    double distance_array[distance_length];//距离滑动平均滤波器数组
    double distance_sum;//距离滑动平均滤波器数据和
    int distance_count;//距离滑动平均滤波器历史数据次数
    bool distance_filter_full_flag;//距离滑动平均滤波器滤波器满标志

    //角度滑动平均滤波器结构
    static const int angle_length = 5;
    double angle_array[angle_length];
    double angle_sum;
    int angle_count;
    bool angle_filter_full_flag;
public:
    void preprocess(const Mat &frame);

    void initialize(int camera);
    void detect_target(Mat &frame, int camera, uint8_t mission_mode, uint8_t is_push, uint8_t is_check_target_in_yellow);
    bool if_get_clamp_position();
    bool if_picked_up();
    bool if_get_putback_position();

    void highlight_yellow_region(Mat &frame);

    float get_target_distance();
    float get_target_angle();
    float get_target_confidence();
    int get_target_type();
    void clear_target_array();
};

#endif //GARBAGESORTINGTROLLEY_DETECT_H
