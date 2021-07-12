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

int main()
{
    //declare and initialize SUB UVC protocol camera
//    VideoCapture cap;
//
//    cap.set(CAP_PROP_FPS, 30);//帧率
//
//    cap.set(CAP_PROP_FRAME_WIDTH, IMAGEWIDTH); //帧宽
//
//    cap.set(CAP_PROP_FRAME_HEIGHT, IMAGEHEIGHT);//帧高
//
//    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));//视频流格式
//
//    cap.open(0);


    //declare and initialize realsense camera
//    realSenseDriver deep_cap;
//
//    deep_cap.InitCam();
//
//    deep_cap.SetCam();
//
//    deep_cap.StartGrab();

    //declare and initialize detector and serial
    Detector detector;

//    Serial serial;

    detector.initialize();

//    serial.init_port();

    //declare and initialize variables
    Mat src,dst;

    Mat src1, dst1;

    ReceiveData receive_data{};

    int mission_state = DETECTING;

    int last_target_type = NOTDEFINEDTYPE;

    bool is_turning;

    src = imread("../res2/1.jpg");
    src1 = imread("../res1/1.jpg");

    stringstream ss;
    int count_res1 = 313;

    while(true)
    {
        cout<<"MISSION STATE"<<mission_state<<endl;
        ss.str("");
        ss<<"../res2/"<<++count_res1<<".jpg";
        src = imread(ss.str());
        cout<<"src path : "<<ss.str()<<endl;
        if(src.empty())
            LOGW("[Warning] : Src is Empty!");
//        if(receive_data.is_turning)
//        {
//            serial.read_data(receive_data);
//            continue;
//        }

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

                    rectangle(src, detector.target_box,Scalar(0,0,255, 1), 2);
                }
                break;
            case NEARING:
                // detector.detect_target(src, NEARING);
                // read from usb
                ss.str("");
                ss<<"../res1/"<<count_res1<<".jpg";
                cout<<"src1 path : "<<ss.str()<<endl;

                src1 = imread(ss.str());
                //detector1.preprocess(src1);
                detector.detect_target(src1,DETECTING);

                if(detector.is_find_target) {
                    LOGW("[MSG] : Claw Area Nearing Target %s", target_types[detector.target_type].c_str());

                    detector.if_get_clamp_position();

                    rectangle(src1, detector.target_box,Scalar(0,255,255, 1), 2);

                    if (detector.is_get_clamp_position) {
                        mission_state = PICKING_UP;
                    }
                }
                else
                {
                    LOGW("[MSG] : Claw Area Not Found Any Target");
                    detector.detect_target(src, NEARING);

                    rectangle(src, detector.target_box,Scalar(0,0,255, 1), 2);
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

//        serial.pack(game_index, detector.get_target_distance(), detector.get_target_angle(),
//                    detector.is_get_clamp_position, detector.is_get_putback_position, mission_state);
//
//        serial.write_data();
//        serial.read_data(receive_data);

        imshow("src", src);
        imshow("src1", src1);

        if(waitKey() == 27)
            break;
    }
    return 0;
}