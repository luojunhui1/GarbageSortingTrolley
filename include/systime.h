//
// Created by root on 2021/6/19.
//

#ifndef MASTER_SYSTEMTIME_H
#define MASTER_SYSTEMTIME_H

typedef double systime;

void getsystime(systime& t);
double getTimeIntervalms(const systime& now, const systime& last);

#if defined(__linux__) || defined(Darwin) || defined(Debian) || defined(Linux)
#include <sys/time.h>
#elif defined(Windows) || defined(_WIN32)
#include <Windows.h>
#else
#error "nonsupport platform."
#endif

#endif //MASTER_SYSTEMTIME_H
