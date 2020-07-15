/**
* base_filter.h
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_FILTER_H
#define BASE_FILTER_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
namespace lidar_algorithm {
namespace points_process {
class BaseFilter
{
public:
    BaseFilter() {}
    virtual ~BaseFilter() {}
public:
    virtual bool init() = 0;
    virtual bool filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr incloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud) = 0;
};
}//namespace points_process
}//namespace lidar_algorithm

#endif // BASE_FILTER_H
