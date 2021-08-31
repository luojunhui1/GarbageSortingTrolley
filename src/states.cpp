//
// Created by root on 2021/6/19.
//
#include "states.h"

/**
 * @brief 初始化状态类各变量
 */
void State::initialize()
{
    distance = 0;
    angle = 0;

    mission_state = DETECTING;
    is_target_found = false;
    is_target_close = false;
    is_target_in_center = false;
    is_get_clamp_position = false;
    is_get_putback_position = false;
    second_target_type = false;

    target_type = NOTDEFINEDTYPE;
    direction = NOTDEFINEDDIRECTION;
    is_push = false;
    is_check_target_in_yellow = false;

    confidence = 0;
}

/**
 * @brief 清除状态类中需要清楚的状态；量，有些状态量如角度、距离由于过滤算法的需求不在此进行处理
 */
void State::clear()
{
    is_target_found = false;
    is_target_close = false;
    is_target_in_center = false;
    is_get_clamp_position = false;
    is_get_putback_position = false;
    second_target_type = NOTDEFINEDTYPE;

    direction = NOTDEFINEDDIRECTION;

    confidence = 0;
}