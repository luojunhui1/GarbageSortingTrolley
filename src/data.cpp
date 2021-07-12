//
// Created by root on 2021/6/21.
//
#include <string>

#include "data.h"

#include <string>
#include <opencv2/opencv.hpp>
uint8_t game_index;

int FRAME_WIDTH = 640;
int FRAME_HEIGHT = 480;

std::string target_types[] = {"NOTDEFINEDTYPE", "BATTERY", "SPITBALL", "PERICARP", "BOTTLE", "CUP"};
std::string cfgPath = "../resource/yolov4-tiny-custom.cfg";
std::string weightPath = "../resource/yolov4-tiny-custom_final.weights";
