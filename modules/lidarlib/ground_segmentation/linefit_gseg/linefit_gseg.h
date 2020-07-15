/**
* linefit_gseg.h
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef LINEFIT_GSEG_H
#define LINEFIT_GSEG_H
#include "modules/lidarlib/interface/base_linefilt_gseg.h"
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_gsegmentation.h"
namespace lidar_algorithm {
namespace points_process {
class LinefitGSeg : public BaseLinefitGSeg
{
public:
    LinefitGSeg(){}
    virtual ~LinefitGSeg() {}
public:
    bool init() override;
    bool gseg(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     std::vector<int>* const glabels) override;
private:
    linefit_gseg::GroundSegmentation _lfgseg;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // LINEFIT_GSEG_H
