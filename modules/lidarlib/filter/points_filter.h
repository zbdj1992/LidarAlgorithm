/**
* points_filter.h
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef POINTS_FILTER_H
#define POINTS_FILTER_H
#include "modules/lidarlib/interface/base_filter.h"
namespace lidar_algorithm {
namespace points_process {
class PointsFilter : public BaseFilter
{
public:
    PointsFilter() {}
    virtual ~PointsFilter() {}
public:
    bool init() override;
    bool filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr incloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud) override;
private:
    bool load_conf(const std::string& file_path);
    double _conf_vg_size;
private:
    bool vg_filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr incloud,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // POINTS_FILTER_H
