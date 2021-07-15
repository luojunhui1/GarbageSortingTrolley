//
// Created by root on 2021/7/11.
//

#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <omp.h>

#include "data.h"
#include "detect.h"
#include "states.h"
#include "serial.h"

#include "realSenseDriver.h"

using namespace std;
using namespace cv;


int main()
{
    //declare and initialize SUB UVC protocol camera
    VideoCapture usb_cap(0);

    usb_cap.set(CAP_PROP_FPS , 30);//高度

    usb_cap.set(CAP_PROP_FRAME_WIDTH , 640);//宽度

    usb_cap.set(CAP_PROP_FRAME_HEIGHT, 480);//高度

    usb_cap.set(CAP_PROP_FOURCC , VideoWriter::fourcc('M', 'J', 'P', 'G'));//高度


    //declare and initialize realsense camera
    realSenseDriver deep_cap;

    deep_cap.InitCam();

    deep_cap.SetCam();

    deep_cap.StartGrab();

    //declare and initialize detector and serial
    Detector usb_detector, deep_detector;

    usb_detector.initialize();

    deep_detector.initialize();

    Serial serial;

    serial.init_port();

    //declare and initialize variables
    Mat deep_src,dst;

    Mat usb_src, dst1;

    deep_cap.Grab(deep_src);
    usb_cap.read(usb_src);

    LOGM("UVC Camera Image Size : %d * %d", usb_src.cols, usb_src.rows);
    LOGM("Realsense Camera Image Size : %d * %d", deep_src.cols, deep_src.rows);

    ReceiveData receive_data{};

    stringstream ss;

    //declare and initialize states
    State state{};
    state.initialize();

    char cur_case;

    while(true)
    {
        LOGM("MISSION : %s", mission_names[state.mission_state].c_str());
        int64 start = getTickCount();
#pragma omp parallel sections default(none) shared(usb_cap, usb_src, usb_detector, deep_cap, deep_src, deep_detector)
        {
#pragma omp section
            {
                usb_cap.read(usb_src);

                usb_detector.detect_target(usb_src, USBCAMERA);
            }
#pragma omp section
            {
                deep_cap.read(deep_src);

                deep_detector.detect_target(deep_src, DEEPCAMERA);
            }
        }

        float time = (getTickCount() - start) / getTickFrequency();
        LOGM("Time : %lf", time);

        state.clear();

        cur_case = 0;
        if(usb_detector.is_find_target)
            cur_case = 0x01;
        if(deep_detector.is_find_target)
            cur_case |= 0x02;

        switch (state.mission_state) {
            case DETECTING:
                if(cur_case == 3)
                {
                    if (usb_detector.target_confidence > deep_detector.target_confidence)
                        cur_case = 1;
                    else
                        cur_case = 2;
                }

                switch (cur_case) {
                    case 0:
                        state.is_target_found = false;
                        break;
                    case 1:
                        state.is_target_found = true;
                        state.is_target_close = true;
                        state.angle = usb_detector.angle;
                        state.distance = usb_detector.distance;
                        state.target_type = usb_detector.target_type;
                        state.mission_state = NEARING2;
                        break;
                    case 2:
                        state.is_target_found = true;
                        state.angle = deep_detector.angle;
                        state.distance = deep_cap.measure(deep_detector.target_box);
                        state.target_type = deep_detector.target_type;
                        state.mission_state = NEARING1;
                        break;
                    default:
                        break;
                }
                break;

            case NEARING1:
                if((cur_case&0x1) > 0)
                {
                    if(state.target_type != usb_detector.target_type)
                        LOGW("Target Changed from %s to %s", target_types[state.target_type].c_str(), target_types[usb_detector.target_type].c_str());

                    state.is_target_found = true;
                    state.is_target_close = true;
                    state.angle = usb_detector.angle;
                    state.distance = usb_detector.distance;
                    state.target_type = usb_detector.target_type;
                    state.mission_state = NEARING2;
                    break;
                }

                if(cur_case >= 2)
                {
                    state.is_target_found = true;
                    state.angle = deep_detector.angle;
                    state.distance = deep_cap.measure(deep_detector.target_box);
                    state.target_type = deep_detector.target_type;
                }
                else if(cur_case == 0)
                {
                    state.is_target_found = false;
                    state.mission_state = DETECTING;
                }

                break;
            case NEARING2:
                if((cur_case&0x1) == 0)
                {
                    state.is_target_found = false;
                    state.mission_state = DETECTING;
                }
                else
                {
                    state.is_target_found = true;

                    usb_detector.if_get_clamp_position();

                    if(fabs(usb_detector.angle) < 10)
                    {
                        state.is_target_in_center = true;

                        if(fabs(usb_detector.distance) < 25)
                        {
                            state.is_get_clamp_position = true;
                            state.mission_state = PICKUP;
                        }
                    }

                }
                break;
            case PICKUP:
                if(!receive_data.is_clampe_complete)
                    continue;
                if(usb_detector.if_picked_up())
                {
                    state.is_clamp_success = true;
                    state.mission_state = PUTBACK;
                }
                else
                {
                    state.is_clamp_success = false;
                    state.mission_state = NEARING2;
                }
                break;
            case PUTBACK:
                if(!receive_data.is_front_area)
                    continue;

                if(receive_data.is_putback_complete)
                {
                    state.mission_state = DETECTING;
                    break;
                }
                deep_detector.preprocess(deep_src);
                deep_detector.if_get_putback_position();

                state.direction = deep_detector.direction;
                state.is_get_putback_position = deep_detector.is_get_putback_position;
                break;
            default:
                break;
        }

        serial.pack(state);
        serial.write_data();

        serial.read_data(receive_data);
#ifdef DEBUG_

        rectangle(deep_src, deep_detector.target_box, Scalar(0, 0, 255));
        rectangle(usb_src, usb_detector.target_box, Scalar(0, 0, 255));

        imshow("deep", deep_src);
        imshow("usb", usb_src);

        if(waitKey(10) == 27)
            break;
#endif

    }
    return 0;
}}