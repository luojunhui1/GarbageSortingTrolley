//
// Created by root on 2021/6/19.
//
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "data.h"
#include "detect.h"

using namespace cv;

int main()
{
    Mat src,dst;

    VideoCapture cap;
    cap.open(0);

    cap>>src;
//    pyrDown(src, src);

    FRAME_WIDTH = src.cols;
    FRAME_HEIGHT = src.rows;

    Detector detector;

    while(true)
    {
        cap>>src;
//        pyrDown(src, src);

        detector.preprocess(src, dst);

//        imshow("src", src);

        if(waitKey(20) == 27)
            break;
    }
    return 0;
}