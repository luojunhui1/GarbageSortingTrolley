//
// Created by root on 2021/6/19.
//

#ifndef MASTER_DATA_H
#define MASTER_DATA_H
#include <string>

#ifdef extern_
    #define extern_ extern
#endif

extern_ int FRAME_WIDTH;
extern_ int FRAME_HEIGHT;
extern_ std::string cfgPath = "../resource/conf.cfg";
extern_ std::string weightPath = "../resource/528.weights";
#endif //MASTER_DATA_H
