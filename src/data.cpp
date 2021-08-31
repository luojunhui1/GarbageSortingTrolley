//
// Created by root on 2021/6/21.
//
#include <string>

#include "data.h"

#include <string>
#include <opencv2/opencv.hpp>
uint8_t game_index;

int FRAME_WIDTH = 640;//frame width, recommended as 640
int FRAME_HEIGHT = 480;//frame height, recommended as 480

/**
 * 将任务分为5个状态和一个未定义状态（NOTDEFINEDMISSION），五个状态分别为：
 * DETECTING: 检测状态，检测当前视野内垃圾并选定目标
 * NEARING1 : 接近状态的第一阶段， 此时垃圾离小车较远，视觉系统主要任务为调整小车行进方向以接近目标垃圾
 * NEARING2 : 接近状态的第二阶段，此时垃圾离小车较近，视觉系统主要任务为调整小车角度和前进距离以使小车到达夹取垃圾位置
 * PICKUP   : 夹取状态， 此时小车正在夹取垃圾，当小车夹取动作完成后，无论夹取是否成功，退出该状态回到NEARING2或PUTBACK
 * PUTBACK  : 放回状态， 此时小车根据下位机获得的全场坐标自行运动到指定垃圾堆放区前，垃圾放回并驶出堆放区并转回后重新回到DETECING状态
 */
std::string mission_names[] = {"NOT DEFINED MISSION", "DETECTING", "NEARING 1", "NEARING 2", "PICKUP", "PUTBACK"};

/**
 * 根据比赛要求，将垃圾分为5个种类和一个未定义种类（NOTDEFINEDTYPE），分别为：
 * BATTERY  : 7号电池，未撕去包装
 * SPITBALL : 纸团， 白色， 5cm直径
 * PERICARP : 橘子皮
 * BOTTLE   : 水瓶， 250ml容量， 未指定品牌
 * CUP      : 水杯， 纸质
 */
std::string target_types[] = {"NOTDEFINEDTYPE", "BATTERY", "SPITBALL", "PERICARP", "BOTTLE", "CUP"};

/**
 * usb相机 yolo cfg文件路径
 */
std::string usb_cfg_path = "../resource/0826_near.cfg";

/**
 * usb相机 yolo weights文件路径
 */
std::string usb_weight_path = "../resource/0826_near.weights";

/**
 * realsense深度相机 yolo weights文件路径
 */
std::string deep_cfg_path = "../resource/0826_far.cfg";

/**
 * realsense深度相机 yolo weights文件路径
 */
std::string deep_weight_path = "../resource/0826_far.weights";

/**
 * 单目测距方案所需参数
 */
std::string param_path = "../resource/param.xml";

/**
 * 近距离距离-像素映射表
 */
std::string distance_map_path = "../resource/distance_map.png";

/**
 * 近距离角度-像素映射表
 */
std::string angle_map_path = "../resource/angle_map.jpg";

/**
 * 用于发送给下位机判断此时视觉上位机是否初始化完成，若完成则置为true，否则置为false
 */
bool is_success_started = false;

/**
 * 用于记录小车执行了几次推垃圾的动作，根据该变量判断小车此时执行到了比赛过程的哪一步，以便于进行下一步动作
 * 当小车执行过5次推垃圾的动作后，通过两个手段一方面保证接下来小车的行动正确执行：
 * 1. 提高垃圾检测的阈值， 减少识别错垃圾的几率
 * 2. 当在DETECTING状态下检测到垃圾时，保证小车至少能朝垃圾方位正确运行1s左右
 * */
int push_count;
