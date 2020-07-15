/**
* base_box_builder.h
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_BOX_BUILDER_H
#define BASE_BOX_BUILDER_H
#include "modules/common/proto/geometry.pb.h"
#include <pcl/point_types.h>
namespace lidar_algorithm {
namespace points_process {
class BaseBoxBuilder
{
public:
    BaseBoxBuilder() {}
    virtual ~BaseBoxBuilder() {}
public:
    virtual bool init() = 0;
    virtual bool boxbuild(const common::proto::Clusters& clusters,
                     common::proto::AABB3Ds* const aabb3ds,
                          common::proto::OBB3Ds* const obb3ds) = 0;
};
}//namespace points_process
}//namespace lidar_algorithm

#endif // BASE_BOX_BUILDER_H
