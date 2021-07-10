//
// Created by root on 2021/6/19.
//
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "data.h"
#include "detect.h"
#include "states.h"
#include "serial.h"

#include "realSenseDriver.h"

using namespace cv;

int main()
{
    //declare and initialize SUB UVC protocol camera
    VideoCapture cap;

    cap.set(CAP_PROP_FPS, 60);//帧率

    cap.set(CAP_PROP_FRAME_WIDTH, IMAGEWIDTH); //帧宽

    cap.set(CAP_PROP_FRAME_HEIGHT, IMAGEHEIGHT);//帧高

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));//视频流格式

    cap.open(0);


    //declare and initialize realsense camera
    realSenseDriver deep_cap;

    deep_cap.InitCam();

    deep_cap.SetCam();

    deep_cap.StartGrab();

    //declare and initialize detector and serial
    Detector detector;

    Serial serial;

    detector.initialize();

    serial.init_port();

    //declare and initialize variables
    Mat src,dst;

    Mat src1, dst1;

    ReceiveData receive_data{};

    angle_map = imread("../resource/angle_map.jpg");
    distance_map = imread("../resource/distance_map.jpg");

    int mission_state = DETECTING;

    int last_target_type = NOTDEFINEDTYPE;

    bool is_turning;

    while(true)
    {
        deep_cap.Grab(src);

        if(receive_data.is_turning)
        {
            serial.read_data(receive_data);
            continue;
        }

//        detector.preprocess(src);

        switch (mission_state) {
            case DETECTING:
                detector.detect_target(src, DETECTING);

                if(detector.is_find_target)
                {
                    mission_state = NEARING;
                    // detect if the target is always he same

                    if(last_target_type != detector.target_type)
                        LOGW("[WARNING] : Target Type Changed from %s to %s", target_types[last_target_type].c_str()
                             , target_types[detector.target_type].c_str());

                    last_target_type = detector.target_type;
                }
                break;
            case NEARING:
                // detector.detect_target(src, NEARING);
                // read from usb
                cap.read(src1);

                //detector1.preprocess(src1);
                detector.detect_target(src1,DETECTING);

                if(detector.is_find_target)
                    LOGW("[MSG] : Nearing Target %s", target_types[detector.target_type].c_str());
                else
                    LOGW("[MSG] : Not Found Any Target");

                detector.if_get_clamp_position();

                if(detector.is_get_clamp_position)
                {
                    mission_state = PICKING_UP;
                }

                break;
            case PICKING_UP:
                detector.if_picked_up();
                if(detector.is_picked_up)
                {
                    mission_state = PUTTING_BACK;
                }
                break;
            case PUTTING_BACK:

                detector.preprocess(src);

                detector.if_get_putback_position();

                if(detector.is_get_putback_position)
                {
                    mission_state = DETECTING;
                }
                break;
            case NOTDEFINEDMISSION:
                LOGW("[WARNING] : Mission has Not Defined or Not Began");
                break;
        }

        serial.pack(game_index, detector.get_target_distance(), detector.get_target_angle(),
                    detector.is_get_clamp_position, detector.is_get_putback_position, mission_state);

        serial.write_data();
        serial.read_data(receive_data);

        imshow("src", src);

        if(waitKey(20) == 27)
            break;
    }
    return 0;
}