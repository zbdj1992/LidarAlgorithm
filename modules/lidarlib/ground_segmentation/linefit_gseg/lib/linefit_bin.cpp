/**
* linefit_bin.cpp
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_bin.h"
#include <limits>
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
Bin::Bin() :
    has_point_(false) {}
void Bin::addPoint(const pcl::PointXYZ& point) {
    addPoint(sqrt(point.x * point.x + point.y * point.y),
             point.z);
}
/*添加点*/
void Bin::addPoint(const double& d, const double& z) {
    has_point_ = true;
    if (z < minz_point_.z) {
        minz_point_.z = z;
        minz_point_.d = d;
    }
}
}//namespace linefit_gseg
}//namespace points_process
}//namespace lidar_algorithm
