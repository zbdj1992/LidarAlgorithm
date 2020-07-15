/**
* ground_filter.h
* Author: zhubin
* Created on: 2020-07-02
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef GROUND_FILTER_H
#define GROUND_FILTER_H
#include "modules/lidarlib/interface/base_ground_seg.h"
namespace lidar_algorithm {
namespace points_process {
class GroundFilter : public BaseGroundFilter
{
public:
    GroundFilter(){}
    virtual ~GroundFilter(){}
public:
    bool init() override;
    bool groundseg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     pcl::PointIndices::Ptr groundp_indices) override;
private:
    bool sac_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                 pcl::PointIndices::Ptr groundp_indices);
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // GROUND_FILTER_H
