//
// Created by root on 2021/7/11.
//

#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "data.h"
#include "detect.h"
#include "states.h"
#include "serial.h"

#include "realSenseDriver.h"

using namespace std;
using namespace cv;

//#define DEBUG_ 1

int main()
{
    //declare and initialize SUB UVC protocol camera
    VideoCapture cap(0);

    cap.set(CAP_PROP_FPS , 30);//高度

    cap.set(CAP_PROP_FRAME_WIDTH , 640);//宽度

    cap.set(CAP_PROP_FRAME_HEIGHT, 480);//高度

    cap.set(CAP_PROP_FOURCC , VideoWriter::fourcc('M','J','P','G'));//高度


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

    deep_cap.Grab(src);
    cap.read(src1);

    LOGM("UVC Camera Image Size : %d * %d", src1.cols, src1.rows);
    LOGM("Realsense Camera Image Size : %d * %d", src.cols, src.rows);

    ReceiveData receive_data{};

    int mission_state = DETECTING;

    int last_target_type = NOTDEFINEDTYPE;

    stringstream ss;

    while(true)
    {
        cout<<"MISSION STATE"<<mission_state<<endl;

        deep_cap.Grab(src);

        if(src.empty())
            LOGW("Src is Empty!");

//        if(receive_data.is_turning)
//        {
//            serial.read_data(receive_data);
//            continue;
//        }

        switch (mission_state) {
            case DETECTING:
                detector.detect_target(src, DETECTING);

                if(detector.is_find_target)
                {
                    mission_state = NEARING;
                    // detect if the target is always he same

                    if(last_target_type != detector.target_type)
                        LOGW("Target Type Changed from %s to %s", target_types[last_target_type].c_str()
                        , target_types[detector.target_type].c_str());

                    last_target_type = detector.target_type;

                    rectangle(src, detector.target_box,Scalar(0,0,255, 1), 2);
                }
                break;
            case NEARING:
                // detector.detect_target(src, NEARING);
                // read from usb
                cap.read(src1);
                //detector1.preprocess(src1);
                detector.detect_target(src1,DETECTING);

                if(detector.is_find_target) {
                    LOGM("Claw Area Nearing Target %s", target_types[detector.target_type].c_str());

                    detector.if_get_clamp_position();

                    rectangle(src1, detector.target_box,Scalar(0,255,255, 1), 2);

                    if (detector.is_get_clamp_position) {
                        mission_state = PICKING_UP;
                    }
                }
                else
                {
                    LOGM("Claw Area Not Found Any Target");
                    detector.detect_target(src, NEARING);

                    rectangle(src, detector.target_box,Scalar(0,0,255, 1), 2);
                }
#ifdef DEBUG_
                pyrDown(src1, src1);
                imshow("src1", src1);
#endif
                break;
            case PICKING_UP:
                detector.if_picked_up();
                if(receive_data.is_clamped && detector.is_picked_up)
                {
                    mission_state = PUTTING_BACK;
                }
                break;
            case PUTTING_BACK:

                if(!receive_data.is_front_area)
                    break;
                detector.preprocess(src);

                detector.if_get_putback_position();

                if(detector.is_get_putback_position)
                {
                    mission_state = DETECTING;
                }
                break;
            case NOTDEFINEDMISSION:
                LOGW("Mission has Not Defined or Not Began");
                break;
        }

        serial.pack(game_index, detector.get_target_distance(), detector.get_target_angle(),
                    detector.is_get_clamp_position, detector.is_get_putback_position, mission_state, detector.target_type, detector.direction);
        serial.write_data();

        serial.read_data(receive_data);
#ifdef DEBUG_
        pyrDown(src, src);

        imshow("src", src);


        if(waitKey() == 27)
           break;
#endif
    }
    return 0;
}