/**
* base_linefit_gseg.h
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_LINEFILT_GSEG_H
#define BASE_LINEFILT_GSEG_H
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
namespace lidar_algorithm {
namespace points_process {
class BaseLinefitGSeg
{
public:
    BaseLinefitGSeg() {}
    virtual ~BaseLinefitGSeg() {}
public:
    virtual bool init() = 0;
    virtual bool gseg(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     std::vector<int>* const glabels) = 0;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // BASE_LINEFILT_GSEG_H
