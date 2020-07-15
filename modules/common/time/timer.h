/**
* time.h
* Author: zhubin
* Created on: 2019-06-04
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef TIMER_H
#define TIMER_H
#include <ctime>
#include <limits>
#include "modules/common/time/timestamp.h"
namespace lidar_algorithm {
namespace common {
namespace time {
//class Timer
//{
//public:
//    Timer() { _start_time = std::clock();}
//     // return elapsed time in seconds,no-thread safe
//    double elapsed() const { return  double(std::clock() - _start_time) / CLOCKS_PER_SEC; }
//    // return minimum value for elapsed()
//    double elapsed_min() const { return double(1)/double(CLOCKS_PER_SEC); }
//    // return estimated maximum value for elapsed()
//    double elapsed_max() const {
//      return (double((std::numeric_limits<std::clock_t>::max)())
//         - double(_start_time)) / double(CLOCKS_PER_SEC);
//    }
//private:
//    std::clock_t _start_time;
//};

class Timer
{
public:
    Timer() { _start_time = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_microseconds);}
     // return elapsed time in seconds,no-thread safe
    double elapsed() const { return  static_cast<double>(common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_microseconds) - _start_time) * 0.001; }
private:
    long long  _start_time;
};
}//namespace time
}//namespace common
}//namespace lidar_algorithm
#endif // TIMER_H
