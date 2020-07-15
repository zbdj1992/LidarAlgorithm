/**
* process.cpp
* Author: zhubin
* Created on: 2018-09-01
* Copyright (c) iRotran. All Rights Reserved
*/
#include "process.h"
#include "modules/framework/conf/parameter.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/common.h"
namespace lidar_algorithm {
namespace framework {
namespace process {
Process::Process(void)
{
    _pclshow = nullptr;
    _points_process = nullptr;
}
Process::~Process(void)
{
    if (_pclshow) {
        delete _pclshow;
    }
    if (_points_process) {
        delete _points_process;
    }
}
int Process::init()
{
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    if (conf_obj->_use_pclshow3d) {
        _pclshow = new pcl_show::PclShow3D;
        if (_pclshow == nullptr) {
            LOG(FATAL)<<"pclshow3d module set failed!";
        }else{
            LOG(INFO)<<"pclshow3d module set sucessed!";
        }
        if (_pclshow->init()) {
            LOG(INFO)<<"pclshow3d module init sucessed!";
        }else{
            LOG(FATAL)<<"pclshow3d module init failed!";
        }
    }
    if (conf_obj->_use_lidarlib) {
        _points_process = new points_process::PointsProcess;
        if (_points_process == nullptr) {
            LOG(FATAL)<<"lidarlib module set failed!";
        }else{
            LOG(INFO)<<"lidarlib module set sucessed!";
        }
        if (_points_process->init()) {
            LOG(INFO)<<"lidarlib module init sucessed!";
        }else{
            LOG(FATAL)<<"lidarlib module init failed!";
        }
    }
	return RET_OK;
}

int Process::process()
{
    init();
    if (_pclshow != nullptr) {
        _pclshow->process();
    }
    if (_points_process != nullptr) {
        _points_process->process();
    }
	return RET_OK;
}

}//namespace process
}//namespace framework
}//namespace lidar_algorithm
