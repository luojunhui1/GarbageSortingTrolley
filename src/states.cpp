//
// Created by root on 2021/6/19.
//
#include "states.h"

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
    is_clamp_success = false;

    target_type = NOTDEFINEDTYPE;
    direction = NOTDEFINEDDIRECTION;

    confidence = 0;
}

void State::clear()
{
    distance = 0;
    angle = 0;

    is_target_found = false;
    is_target_close = false;
    is_target_in_center = false;
    is_get_clamp_position = false;
    is_get_putback_position = false;
    is_clamp_success = false;

    target_type = NOTDEFINEDTYPE;
    direction = NOTDEFINEDDIRECTION;

    confidence = 0;
}