/**
* base_cluster.h
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef BASE_CLUSTER_H
#define BASE_CLUSTER_H
#include "modules/common/proto/geometry.pb.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
namespace lidar_algorithm {
namespace points_process {
class BaseCluster
{
public:
    BaseCluster() {}
    virtual ~BaseCluster() {}
public:
    virtual bool init() = 0;
    virtual bool cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     common::proto::Clusters* const clusters) = 0;
};
}//namespace points_process
}//namespace lidar_algorithm

#endif // BASE_CLUSTER_H
