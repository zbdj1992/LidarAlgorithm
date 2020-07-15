/**
* data.cpp
* Author: zhubin
* Created on: 2019-05-23
* Copyright (c) iRotran. All Rights Reserved
*/
#include "data.h"
#include "modules/manager/mutex/mutexmanager.h"
#include "modules/common/common.h"
#include "modules/framework/conf/parameter.h"
#include "modules/common/log/glogutil.h"
namespace lidar_algorithm {
namespace framework {
namespace data {
using namespace manager::mutex;
Data::Data()
{
    _lidardriver = nullptr;
}
Data::~Data()
{
    if (_lidardriver) {
        delete _lidardriver;
    }
}
int Data::init()
{
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    if (conf_obj->_use_lidar) {
        _lidardriver = new drivers::lidar::LidarDriver;
        if (_lidardriver == nullptr) {
            LOG(FATAL)<<"lidar driver set failed!";
        }else{
            LOG(INFO)<<"lidar driver set sucessed!";
        }
        _lidardriver->init(&_pointcloud);
    }
    return RET_OK;
}
int Data::process()
{
    init();
    return RET_OK;
}
common::proto::PointCloud Data::get_one_pointcloud()
{
    {//读锁定，shared_lock
        MutexManager *mutexmanager_obj = MutexManager::get_obj();
        boost::shared_lock<boost::shared_mutex> lidardata_read(mutexmanager_obj->_lidar_data_shmutex);    //读锁定，shared_lock
        return _pointcloud;
    }//读锁定，结束
}
bool Data::set_one_lidarlib_data(const proto::LidarLibData& lidarlib_data)
{
    {//写锁定，unique_lock
        MutexManager *mutexmanager_obj = MutexManager::get_obj();
        boost::unique_lock<boost::shared_mutex> lidarlibdata_write(mutexmanager_obj->_lidarlib_data_shmutex);    //写锁定，unique_lock
        _lidarlibdata = lidarlib_data;
    }//写锁定,结束
    return true;
}
proto::LidarLibData Data::get_one_lidarlib_data()
{
    {//读锁定，shared_lock
        MutexManager *mutexmanager_obj = MutexManager::get_obj();
        boost::shared_lock<boost::shared_mutex> lidarlibdata_read(mutexmanager_obj->_lidarlib_data_shmutex);    //读锁定，shared_lock
        return _lidarlibdata;
    }//读锁定，结束
}
}//namespace data
}//namespace framework
}//namespace lidar_algorithm
