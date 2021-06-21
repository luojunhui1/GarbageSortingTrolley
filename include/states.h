//
// Created by root on 2021/6/19.
//

#ifndef GARBAGESORTINGTROLLEY_STATES_H
#define GARBAGESORTINGTROLLEY_STATES_H

#include "defs.h"

class State
{
private:
    int index;

    int x;
    int y;
    float angle;

    int mission_state;
    int clamp_state;
    int target_type;
    float coefficience;
    bool is_target_pickup;

    bool is_target_detected;
    bool is_position_suitable;

public:
    void set_index(const int index);

    void set_x(const int x);
    void set_y(const int y);
    void set_angle(const float angle);

    void set_mission_state(const int mission_state);
    void set_clamp_state(const int clamp_state);

    void set_target_type(const int target_type, const double coefficience);

    void get_index(int &index);
    void get_xy(int &x, int &y);
    void get_clamp_state(int &clamp_state);
    void get_mission_state(int &mission_state);

    void get_target_type(int &target_type, double &coefficience);

    void initialize();
};
#endif //GARBAGESORTINGTROLLEY_STATES_H
