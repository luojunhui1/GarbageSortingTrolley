//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_DETECT_H
#define GARBAGESORTINGTROLLEY_DETECT_H

#include <vector>

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
    Mat gray[4];
    Mat gray_binary;

    Mat input_blob;
    Net net;
    std::vector<String> out_names;
    std::vector<Mat> outs;

    Point class_id_point;
    double confidence;

    vector<Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;
    vector<int> indices;

    Mat angle_map;
    Mat distance_map;

    double x, y, z;
    float distance;
    float angle;

public:
    bool is_find_target;
    int target_type;
    double target_confidence;
    uint8_t is_get_clamp_position;
    uint8_t is_picked_up;
    uint8_t is_get_putback_position;
    uint8_t direction;
    Rect target_box;

    RotatedRect possibleArea;

private:
    static const int distance_length = 20;

    double distance_array[distance_length];
    double distance_sum;
    int distance_count;
    bool distance_filter_full_flag;

    static const int angle_length = 50;
    double angle_array[angle_length];
    double angle_sum;
    int angle_count;
    bool angle_filter_full_flag;

public:
    void preprocess(const Mat &frame);
    void initialize();
    void detect_target(const Mat &frame, int mission_state);
    void if_get_clamp_position();
    void if_picked_up();
    void if_get_putback_position();

    float get_target_distance();
    float get_target_angle();
};

#endif //GARBAGESORTINGTROLLEY_DETECT_H
