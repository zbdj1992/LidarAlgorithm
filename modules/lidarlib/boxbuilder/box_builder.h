/**
* box_builder.h
* Author: zhubin
* Created on: 2020-06-28
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BOX_BUILDER_H
#define BOX_BUILDER_H
#include "modules/lidarlib/interface/base_box_builder.h"
#include "modules/common/util/pc_format_conv/pcformat_conv.h"
namespace lidar_algorithm {
namespace points_process {
class BoxBuilder : public BaseBoxBuilder
{
public:
    BoxBuilder(){}
    virtual ~BoxBuilder() {}
public:
    bool init() override;
    bool boxbuild(const common::proto::Clusters& clusters,
                     common::proto::AABB3Ds* const aabb3ds,
                          common::proto::OBB3Ds* const obb3ds) override;
private:
    common::util::PCFormatConv _pcformatconv;
private:
    bool get_box(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster_cloud,
                 common::proto::AABB3D* const aabb3d,
                 common::proto::OBB3D* const obb3d);

};
}//namespace points_process
}//namespace lidar_algorithm
#endif // BOX_BUILDER_H
