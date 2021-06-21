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

#include "defs.h"
#include "data.h"
#include "log.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;

class Detector
{
private:
    Mat channels[3];
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

    Rect target_box;
    int target_type;
    double target_coefficience;

public:
    void preprocess(Mat &src, Mat &dst);
    bool initialize();
    bool detect_target(Mat &frame);
};

#endif //GARBAGESORTINGTROLLEY_DETECT_H
