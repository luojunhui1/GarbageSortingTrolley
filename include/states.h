//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_STATES_H
#define GARBAGESORTINGTROLLEY_STATES_H

#include <iostream>
#include "defs.h"

class State
{
public:
    float distance;
    float angle;

    std::uint8_t mission_state;
    std::uint8_t is_target_found;
    std::uint8_t is_target_close;
    std::uint8_t is_target_in_center;
    std::uint8_t is_get_clamp_position;
    std::uint8_t is_get_putback_position;
    std::uint8_t is_clamp_success;

    std::uint8_t target_type;
    std::uint8_t direction;

    float confidence;

public:
    void initialize();
    void clear();
};
#endif //GARBAGESORTINGTROLLEY_STATES_H
