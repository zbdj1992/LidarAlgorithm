/**
* base_post_processing.h
* Author: zhubin
* Created on: 2020-06-28
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_POST_PROCESSING_H
#define BASE_POST_PROCESSING_H
#include "modules/common/proto/geometry.pb.h"
namespace lidar_algorithm {
namespace points_process {
class BasePostProcessing
{
public:
    BasePostProcessing() {}
    virtual ~BasePostProcessing() {}
public:
    virtual bool init() = 0;
    virtual bool cluster_merge(const std::vector<std::vector<common::proto::Point3D>>& inclusters,
                               std::vector<std::vector<common::proto::Point3D>>& outclusters) = 0;
};

}//namespace points_process
}//namespace lidar_algorithm
#endif // BASE_POST_PROCESSING_H
