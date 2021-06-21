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

using namespace cv;

int main()
{
    Mat src,dst;

    VideoCapture cap;
    cap.open(0);
    cap>>src;

    FRAME_WIDTH = src.cols;
    FRAME_HEIGHT = src.rows;

    Detector detector;
    Serial serial;
    ReceiveData receive_data{};

    detector.initialize();
    serial.init_port();

    int mission_state = DETECTING;
    int last_target_type = NOTDEFINEDTYPE;
    bool is_turning;

    while(true)
    {
        cap>>src;

        if(receive_data.is_turning)
        {
            serial.read_data(receive_data);
            continue;
        }

        detector.preprocess(src);
        detector.detect_target(src);

        switch (mission_state) {
            case DETECTING:
                if(detector.is_find_target)
                {
                    mission_state = NEARING;
                    // detect if the target is always he same
                    if(last_target_type == NOTDEFINEDTYPE)
                        last_target_type = detector.target_type;
                    else
                    {
                        if(last_target_type != detector.target_type)
                            LOGW("[WARNING] : Target Type Changed");
                    }
                }
                break;
            case NEARING:
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
                detector.if_get_putback_position();
                if(detector.is_get_putback_position)
                {
                    mission_state = DETECTING;
                }
                break;
            case NOTDEFINEDMISSION:
                LOGW("[WARNING] : Mission Has Not Defined or Not Began");
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