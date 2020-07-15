/**
* base_ground_seg.h
* Author: zhubin
* Created on: 2020-07-02
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_GROUND_SEG_H
#define BASE_GROUND_SEG_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
namespace lidar_algorithm {
namespace points_process {
class BaseGroundFilter
{
public:
    BaseGroundFilter() {}
    virtual ~BaseGroundFilter() {}
public:
    virtual bool init() = 0;
    virtual bool groundseg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     pcl::PointIndices::Ptr groundp_indices) = 0;
};

}//namespace points_process
}//namespace lidar_algorithm
#endif // BASE_GROUND_SEG_H
