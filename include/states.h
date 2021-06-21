//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_STATES_H
#define GARBAGESORTINGTROLLEY_STATES_H

#include <iostream>
#include "defs.h"

class State
{
private:
    int index;

    int x;
    int y;
    float angle;

    std::uint8_t mission_state;
    std::uint8_t clamp_state;
    int target_type;
    float confidence;
    bool is_target_pickup;

    bool is_target_detected;
    bool is_position_suitable;

public:
    void set_index(int index_);

    void set_x(int x);
    void set_y(int y);
    void set_angle(float angle);

    void set_mission_state(int mission_state);
    void set_clamp_state(int clamp_state);

    void set_target_type(int target_type, double confidence);

    int get_index() const;
    int get_x() const;
    int get_y() const;
    int get_clamp_state() const;
    int get_mission_state() const;

    int get_target_type() const;
    double get_target_confidence() const;

    void initialize();
};
#endif //GARBAGESORTINGTROLLEY_STATES_H
