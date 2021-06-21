//
// Created by root on 2021/6/19.
//
#include "states.h"

void State::set_index(const int index) { this->index = index;}
void State::set_x(const int x) {this->x = x;}
void State::set_y(const int y) {this->y = y;}
void State::set_angle(const float angle) {this->angle = angle;}
void State::set_clamp_state(const int clamp_state) {this->clamp_state = clamp_state;}
void State::set_mission_state(const int mission_state) {this->mission_state = mission_state;}
void State::set_target_type(const int target_type, const double confidence) {this->target_type = target_type;
                            this->confidence = confidence;}

int State::get_index() const {return index;}
int State::get_x() const {return x;}
int State::get_y() const {return y;}
int State::get_clamp_state() const {return clamp_state;}
int State::get_mission_state() const {return mission_state;}
int State::get_target_type() const {return target_type;}
double State::get_target_confidence() const {return confidence;}

void State::initialize()
{
    index = 0;

    x = y = 0;
    angle = 0;

    clamp_state = false;

    mission_state = NOTDEFINEDMISSION;

    target_type = NOTDEFINEDTYPE;
    confidence = 0;


}