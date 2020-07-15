/**
* ceu_cluster.cpp
* Author: zhubin
* Created on: 2020-07-11
* Copyright (c) iRotran. All Rights Reserved
*/
#include "ceu_cluster.h"
#include "modules/lidarlib/segmentation/cluster_postprocessing/cluster_postprocessing.h"
namespace lidar_algorithm {
namespace points_process {
bool CEuCluster::init()
{
    //load conf
    //load_conf(conf_path);
    //load post processing method
    _postprocessing.reset(new ClusterPostProcessing);
    _postprocessing->init();
    return true;
}
bool CEuCluster::cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 common::proto::Clusters* const clusters)
{
    std::vector<std::vector<common::proto::Point3D>> origin_clusters;
    std::vector<std::vector<common::proto::Point3D>> merge_clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    //eu_cluster(cloud,cluster_indices);
    //get_clusters(cloud,cluster_indices,origin_clusters);
    //post processing
//    if (_conf_use_cluster_merge){
//        _postprocessing->cluster_merge(origin_clusters,merge_clusters);
//        public_clusters(merge_clusters,clusters);
//    }else{
//        public_clusters(origin_clusters,clusters);
//    }
//    if (_conf_use_log_debug) {
//        LOG(WARNING)<<"origin_clusters num: "<<origin_clusters.size()<<" merge_clusters num: "
//                   <<merge_clusters.size()<<" clusters num: "<<clusters->clusters_size();
//    }
    return true;
}

}//namespace points_process
}//namespace lidar_algorithm
