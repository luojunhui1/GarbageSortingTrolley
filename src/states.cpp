//
// Created by root on 2021/6/19.
//
#include "states.h"

void State::set_index(const int index) {this->index = index;}
void State::set_x(const int x) {this->x = x;}
void State::set_y(const int y) {this->y = y;}
void State::set_angle(const float angle) {this->angle = angle;}
void State::set_clamp_state(const int clamp_state) {this->clamp_state = clamp_state;}
void State::set_mission_state(const int mission_state) {this->mission_state = mission_state;}
void State::set_target_type(const int target_type, const double coefficience) {this->target_type = target_type;
                            this->coefficience = coefficience;}

void State::get_index(int &index) {index = this->index;}
void State::get_xy(int &x, int &y) {x = this->x;y = this->y;}
void State::get_clamp_state(int &clamp_state) {clamp_state = this->clamp_state;}
void State::get_mission_state(int &mission_state) {mission_state = this->mission_state;}
void State::get_target_type(int &target_type, double &coefficience) {target_type = this->target_type;
                            coefficience = this->coefficience;}

void State::initialize()
{
    index = 0;

    x = y = 0;
    angle = 0;

    clamp_state = false;

    mission_state = NOTDEFINEDMISSION;

    target_type = NOTDEFINEDTYPE;
    coefficience = 0;


}