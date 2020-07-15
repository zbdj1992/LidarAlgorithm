/**
* timestamp.h
* Author: zhubin
* Created on: 2019-06-05
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef TIMESTAMP_H
#define TIMESTAMP_H
#include <boost/chrono.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
namespace lidar_algorithm {
namespace common {
namespace time {
typedef enum{
    _hours = 0,
    _minutes,
    _seconds,
    _milliseconds,
    _microseconds,
    _nanoseconds,
}TimeFormat_EN;
typedef enum {
    _resolution_second = 0,
    _resolution_microsecond,
}DateResolution_EN;
class TimeStamp
{
public:
    TimeStamp();
public:
    long long get_now_timestamp_system(TimeFormat_EN timeformat);
    long long get_now_timestamp_steady(TimeFormat_EN timeformat);
    void get_now_date(std::string& str_data_time,DateResolution_EN date_resolution);
public:
    static TimeStamp* get_obj() {
        static TimeStamp timestamp_obj;
        return &timestamp_obj;
    }
};
}//namespace time
}//namespace common
}//namespace lidar_algorithm
#endif // TIMESTAMP_H
