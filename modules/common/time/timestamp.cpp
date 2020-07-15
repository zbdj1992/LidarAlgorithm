/**
* timestamp.cpp
* Author: zhubin
* Created on: 2019-06-05
* Copyright (c) iRotran. All Rights Reserved
*/
#include "timestamp.h"
namespace lidar_algorithm {
namespace common {
namespace time {
TimeStamp::TimeStamp()
{

}
long long TimeStamp::get_now_timestamp_system(TimeFormat_EN timeformat)
{
    long long now_timestamp = 0;
    boost::chrono::hours hour;
    boost::chrono::minutes min;
    boost::chrono::seconds sec;
    boost::chrono::milliseconds milli;
    boost::chrono::microseconds micro;
    boost::chrono::nanoseconds nano;
    boost::chrono::system_clock::duration dur = boost::chrono::system_clock::now().time_since_epoch();
    switch (timeformat) {
    case _hours:
        hour = boost::chrono::duration_cast<boost::chrono::hours>(dur);
        now_timestamp = min.count();
        break;
    case _minutes:
        min = boost::chrono::duration_cast<boost::chrono::minutes>(dur);
        now_timestamp = min.count();
        break;
    case _seconds:
        sec = boost::chrono::duration_cast<boost::chrono::seconds>(dur);
        now_timestamp = sec.count();
        break;
    case _milliseconds:
        milli = boost::chrono::duration_cast<boost::chrono::milliseconds>(dur);
        now_timestamp = milli.count();
        break;
    case _microseconds:
        micro = boost::chrono::duration_cast<boost::chrono::microseconds>(dur);
        now_timestamp = micro.count();
        break;
    case _nanoseconds:
        nano = boost::chrono::duration_cast<boost::chrono::microseconds>(dur);
        now_timestamp = nano.count();
        break;
    default:
        milli = boost::chrono::duration_cast<boost::chrono::milliseconds>(dur);
        now_timestamp = milli.count();
        break;
    }
    return now_timestamp;
}
long long TimeStamp::get_now_timestamp_steady(TimeFormat_EN timeformat)
{
    long long now_timestamp = 0;
    boost::chrono::hours hour;
    boost::chrono::minutes min;
    boost::chrono::seconds sec;
    boost::chrono::milliseconds milli;
    boost::chrono::microseconds micro;
    boost::chrono::nanoseconds nano;
    boost::chrono::steady_clock::duration dur = boost::chrono::steady_clock::now().time_since_epoch();
    switch (timeformat) {
    case _hours:
        hour = boost::chrono::duration_cast<boost::chrono::hours>(dur);
        now_timestamp = min.count();
        break;
    case _minutes:
        min = boost::chrono::duration_cast<boost::chrono::minutes>(dur);
        now_timestamp = min.count();
        break;
    case _seconds:
        sec = boost::chrono::duration_cast<boost::chrono::seconds>(dur);
        now_timestamp = sec.count();
        break;
    case _milliseconds:
        milli = boost::chrono::duration_cast<boost::chrono::milliseconds>(dur);
        now_timestamp = milli.count();
        break;
    case _microseconds:
        micro = boost::chrono::duration_cast<boost::chrono::microseconds>(dur);
        now_timestamp = micro.count();
        break;
    case _nanoseconds:
        nano = boost::chrono::duration_cast<boost::chrono::microseconds>(dur);
        now_timestamp = nano.count();
        break;
    default:
        milli = boost::chrono::duration_cast<boost::chrono::milliseconds>(dur);
        now_timestamp = milli.count();
        break;
    }
    return now_timestamp;
}
void TimeStamp::get_now_date(std::string& str_data_time,DateResolution_EN date_resolution)
{
    boost::posix_time::ptime now_date_time;
    switch (date_resolution) {
    case _resolution_second:
        now_date_time = boost::posix_time::second_clock::local_time();
        break;
    case _resolution_microsecond:
        now_date_time = boost::posix_time::microsec_clock::local_time();
        break;
    default:
        now_date_time = boost::posix_time::second_clock::local_time();
        break;
    }
    std::string str_date = boost::gregorian::to_iso_extended_string(now_date_time.date());
    std::string str_time_Of_day = boost::posix_time::to_simple_string(now_date_time.time_of_day());
    std::replace(str_time_Of_day.begin(),str_time_Of_day.end(),'-','.');
    std::replace(str_time_Of_day.begin(),str_time_Of_day.end(),':','.');
    str_data_time = str_date + "." + str_time_Of_day;
}
}//namespace time
}//namespace common
}//namespace lidar_algorithm
