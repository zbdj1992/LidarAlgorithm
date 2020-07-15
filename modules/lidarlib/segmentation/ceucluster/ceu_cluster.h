/**
* ceu_cluster.h
* Author: zhubin
* Created on: 2020-07-11
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef CEU_CLUSTER_H
#define CEU_CLUSTER_H
#include "modules/lidarlib/interface/base_cluster.h"
#include "modules/lidarlib/interface/base_post_processing.h"
namespace lidar_algorithm {
namespace points_process {
class CEuCluster : public BaseCluster
{
public:
    CEuCluster(){}
    virtual ~CEuCluster(){}
public:
    bool init() override;
    bool cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     common::proto::Clusters* const clusters) override;
private:
    bool ceu_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
                std::vector<pcl::PointIndices>& cluster_indices);
    bool get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
            const std::vector<pcl::PointIndices>& cluster_indices,
                      std::vector<std::vector<common::proto::Point3D>>& clusters);
    bool public_clusters(const std::vector<std::vector<common::proto::Point3D>>& clusters,
                         common::proto::Clusters* const public_clusters);
private:
    std::unique_ptr<BasePostProcessing> _postprocessing;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // CEU_CLUSTER_H
