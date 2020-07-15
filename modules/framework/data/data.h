/**
* data.h
* Author: zhubin
* Created on: 2019-05-23
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef DATA_H
#define DATA_H
#include "modules/common/proto/geometry.pb.h"
#include "modules/drivers/lidar/lidar_driver.h"
#include "modules/framework/data/proto/lidarlib_modu_result.pb.h"
namespace lidar_algorithm {
namespace framework {
namespace data {
class Data
{
public:
    Data(void);
    ~Data(void);
public:
    int init();
    int process();
public:
    static Data* get_obj() {
        static Data data_obj;
        return &data_obj;
    }
public:
    bool set_one_lidarlib_data(const proto::LidarLibData& lidarlib_data);
public:
    common::proto::PointCloud get_one_pointcloud();
    proto::LidarLibData get_one_lidarlib_data();
private:
    drivers::lidar::LidarDriver *_lidardriver;
private:
    common::proto::PointCloud _pointcloud;
    proto::LidarLibData _lidarlibdata;
};
}//namespace data
}//namespace framework
}//namespace lidar_algorithm
#endif // DATA_H
