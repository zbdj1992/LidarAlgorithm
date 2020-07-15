/**
* reg_cluster.h
* Author: zhubin
* Created on: 2020-06-29
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef REG_CLUSTER_H
#define REG_CLUSTER_H
#include "modules/lidarlib/interface/base_cluster.h"
#include "modules/lidarlib/interface/base_post_processing.h"
namespace lidar_algorithm {
namespace points_process {
class RegCluster : public BaseCluster
{
public:
    RegCluster(){}
    virtual ~RegCluster(){}
public:
    bool init() override;
    bool cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                     common::proto::Clusters* const clusters) override;
private:
    bool reg_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
                std::vector<pcl::PointIndices>& cluster_indices);
    bool get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
            const std::vector<pcl::PointIndices>& cluster_indices,
                      std::vector<std::vector<common::proto::Point3D>>& clusters);
    bool public_clusters(const std::vector<std::vector<common::proto::Point3D>>& clusters,
                         common::proto::Clusters* const public_clusters);
private:
    std::unique_ptr<BasePostProcessing> _postprocessing;
private:
    bool load_conf(const std::string& file_path);
    int  _conf_nor_k_search;
    int _conf_reg_cluster_min_size;
    int _conf_reg_cluster_max_size;
    int _conf_reg_number_neighbours;
    double _conf_reg_smoothness_thr;
    double _conf_reg_curvature_thr;
    int _conf_use_cluster_merge;
    int _conf_use_log_debug;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // REG_CLUSTER_H
