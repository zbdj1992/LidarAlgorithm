/**
* glogutil.cpp
* Author: zhubin
* Created on: 2019-05-31
* Copyright (c) iRotran. All Rights Reserved
*/
#include "glogutil.h"
#include "modules/common/common.h"
#include <stdlib.h>
namespace lidar_algorithm {
namespace common {
namespace log {
GlogUtil::GlogUtil()
{

}
GlogUtil::~GlogUtil()
{
    google::ShutdownGoogleLogging();  //GLOG内存清理
}
int GlogUtil::init(char* program,const std::string log_path)
{
    std::string str_cmd = "mkdir -p ";
    str_cmd += log_path;
    system(str_cmd.c_str());
    google::InitGoogleLogging(program);
    #ifdef DEBUG_MODE
       google::SetStderrLogging(google::GLOG_INFO); //设置级别高于 google::INFO 的日志同时输出到屏幕
    #else
       google::SetStderrLogging(google::GLOG_FATAL);//设置级别高于 google::FATAL 的日志同时输出到屏幕
    #endif
    FLAGS_colorlogtostderr=true;    //设置输出到屏幕的日志显示相应颜色
    std::string base_info = log_path;
    base_info += "/INFO_";
    google::SetLogDestination(google::INFO,base_info.c_str()); //设置 google::INFO 级别的日志存储路径和文件名前缀
    std::string base_warn = log_path;
    base_warn += "/WARNING_";
    google::SetLogDestination(google::WARNING,base_warn.c_str());   //设置 google::WARNING 级别的日志存储路径和文件名前缀
    std::string base_error = log_path;
    base_error += "/ERROR_";
    google::SetLogDestination(google::ERROR,base_error.c_str());   //设置 google::ERROR 级别的日志存储路径和文件名前缀
    std::string base_fatal = log_path;
    base_fatal += "/FATAL_";
    google::SetLogDestination(google::FATAL,base_fatal.c_str());   //设置 google::FATAL 级别的日志存储路径和文件名前缀
    FLAGS_max_log_size =100;  //最大日志大小为 100MB
    FLAGS_stop_logging_if_full_disk = true;     //当磁盘被写满时，停止日志输出
    FLAGS_logbufsecs =0;        //缓冲日志输出，默认为30秒，此处改为立即输出
    google::SetLogFilenameExtension("LidarAlgorithm_");     //设置文件名扩展，如平台？或其它需要区分的信息
    google::InstallFailureSignalHandler();      //捕捉 core dumped
    return RET_OK;
}
}//namespace log
}//namespace common
}//namespace lidar_algorithm
