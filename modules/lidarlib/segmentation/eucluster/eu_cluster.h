/**
* eu_cluster.h
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef EU_CLUSTER_H
#define EU_CLUSTER_H
#include "modules/lidarlib/interface/base_cluster.h"
#include "modules/lidarlib/interface/base_post_processing.h"
namespace lidar_algorithm {
namespace points_process {
class EuCluster : public BaseCluster
{
public:
    EuCluster(){}
    virtual ~EuCluster() {}
public:
    bool init() override;
    bool cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     common::proto::Clusters* const clusters) override;
private:
    bool eu_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
                std::vector<pcl::PointIndices>& cluster_indices);
    bool get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
            const std::vector<pcl::PointIndices>& cluster_indices,
                      common::proto::Clusters* const clusters);
    bool get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
            const std::vector<pcl::PointIndices>& cluster_indices,
                      std::vector<std::vector<common::proto::Point3D>>& clusters);
    bool public_clusters(const std::vector<std::vector<common::proto::Point3D>>& clusters,
                         common::proto::Clusters* const public_clusters);
private:
    std::unique_ptr<BasePostProcessing> _postprocessing;
private:
    bool load_conf(const std::string& file_path);
    double _conf_ec_cluster_tolerence;
    int _conf_ec_cluster_min_size;
    int _conf_ec_cluster_max_size;
    int _conf_use_cluster_merge;
    int _conf_use_log_debug;

};
}//namespace points_process
}//namespace lidar_algorithm
#endif // EU_CLUSTER_H
