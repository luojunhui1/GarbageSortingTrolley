//
// Created by root on 2021/6/21.
//
#ifndef MASTER_DATA_H
#define MASTER_DATA_H
#include <string>
#include <opencv2/opencv.hpp>

extern uint8_t game_index;

extern int FRAME_WIDTH;
extern int FRAME_HEIGHT;

extern std::string mission_names[];
extern std::string target_types[];

extern std::string usb_cfg_path;
extern std::string usb_weight_path;

extern std::string deep_cfg_path;
extern std::string deep_weight_path;

extern std::string param_path;
extern std::string distance_map_path;
extern std::string angle_map_path;

#endif //MASTER_DATA_H
