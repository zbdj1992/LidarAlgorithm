/**
* linefit_gseg.cpp
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#include "linefit_gseg.h"
#include "modules/common/readyaml/readyaml.h"
#include "modules/common/log/glogutil.h"
namespace lidar_algorithm {
namespace points_process {
bool LinefitGSeg::init()
{
    _lfgseg.init();
    return true;
}
bool LinefitGSeg::gseg(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 std::vector<int>* const glabels)
{
    glabels->clear();
    if (cloud->empty()) {
        return false;
    }
    _lfgseg.segment(*cloud,glabels);
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
