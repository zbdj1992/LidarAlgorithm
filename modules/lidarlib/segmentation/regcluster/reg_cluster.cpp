/**
* reg_cluster.cpp
* Author: zhubin
* Created on: 2020-06-29
* Copyright (c) iRotran. All Rights Reserved
*/
#include "reg_cluster.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include "modules/lidarlib/segmentation/cluster_postprocessing/cluster_postprocessing.h"
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
namespace lidar_algorithm {
namespace points_process {
const std::string conf_path = "../modules/lidarlib/conf/reg_cluster_conf.yaml";
bool RegCluster::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"nor_k_search",this->_conf_nor_k_search,5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"nor_k_search conf failed!";
    }else{
        LOG(INFO)<<"nor_k_search = "<<this->_conf_nor_k_search;
    }
    readyaml.parse_yaml(parma_node,"reg_cluster_min_size",this->_conf_reg_cluster_min_size,5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"reg_cluster_min_size conf failed!";
    }else{
        LOG(INFO)<<"reg_cluster_min_size = "<<this->_conf_reg_cluster_min_size;
    }
    readyaml.parse_yaml(parma_node,"reg_cluster_max_size",this->_conf_reg_cluster_max_size,5000);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"reg_cluster_max_size conf failed!";
    }else{
        LOG(INFO)<<"reg_cluster_max_size = "<<this->_conf_reg_cluster_max_size;
    }
    readyaml.parse_yaml(parma_node,"reg_number_neighbours",this->_conf_reg_number_neighbours,10);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"reg_number_neighbours conf failed!";
    }else{
        LOG(INFO)<<"reg_number_neighbours = "<<this->_conf_reg_number_neighbours;
    }
    readyaml.parse_yaml(parma_node,"reg_smoothness_thr",this->_conf_reg_smoothness_thr,3.0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"reg_smoothness_thr conf failed!";
    }else{
        LOG(INFO)<<"reg_smoothness_thr = "<<this->_conf_reg_smoothness_thr;
    }
    readyaml.parse_yaml(parma_node,"reg_curvature_thr",this->_conf_reg_curvature_thr,1.0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"reg_curvature_thr conf failed!";
    }else{
        LOG(INFO)<<"reg_curvature_thr = "<<this->_conf_reg_curvature_thr;
    }
    readyaml.parse_yaml(parma_node,"use_cluster_merge",this->_conf_use_cluster_merge,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_cluster_merge conf failed!";
    }else{
        LOG(INFO)<<"use_cluster_merge = "<<this->_conf_use_cluster_merge;
    }
    readyaml.parse_yaml(parma_node,"use_log_debug",this->_conf_use_log_debug,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_log_debug conf failed!";
    }else{
        LOG(INFO)<<"use_log_debug = "<<this->_conf_use_log_debug;
    }
    return true;
}
bool RegCluster::init()
{
    //load conf
    load_conf(conf_path);
    //load post processing method
    _postprocessing.reset(new ClusterPostProcessing);
    _postprocessing->init();
    return true;
}
bool RegCluster::cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 common::proto::Clusters* const clusters)
{
    std::vector<std::vector<common::proto::Point3D>> origin_clusters;
    std::vector<std::vector<common::proto::Point3D>> merge_clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    reg_cluster(cloud,cluster_indices);
    get_clusters(cloud,cluster_indices,origin_clusters);
    //post processing
    if (_conf_use_cluster_merge){
        _postprocessing->cluster_merge(origin_clusters,merge_clusters);
        public_clusters(merge_clusters,clusters);
    }else{
        public_clusters(origin_clusters,clusters);
    }
    if (_conf_use_log_debug) {
        LOG(WARNING)<<"origin_clusters num: "<<origin_clusters.size()<<" merge_clusters num: "
                   <<merge_clusters.size()<<" clusters num: "<<clusters->clusters_size();
    }
    return true;
}
bool RegCluster::reg_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr  incloud,
                             std::vector<pcl::PointIndices> &cluster_indices)
{
    cluster_indices.clear();
    size_t cloud_num = incloud->size();
    if (cloud_num == 0) {
        return false;
    }
    //NormalEstimation
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (incloud);
    normal_estimator.setKSearch (_conf_nor_k_search);
    normal_estimator.compute (*normals);
    //RegionGrowing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (_conf_reg_cluster_min_size);
    reg.setMaxClusterSize (_conf_reg_cluster_max_size);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (_conf_reg_number_neighbours);
    reg.setInputCloud (incloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (_conf_reg_smoothness_thr / 180.0 * M_PI);
    reg.setCurvatureThreshold (_conf_reg_curvature_thr);
    reg.extract (cluster_indices);
    return true;
}
bool RegCluster::get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
                  std::vector<std::vector<common::proto::Point3D>>& clusters)
{
    int clusters_num = cluster_indices.size();
    clusters.resize(clusters_num);
    int cnt = 0;
    common::proto::Point3D onepoint;
    for (std::vector<pcl::PointIndices>::const_iterator iter_k = cluster_indices.begin();iter_k != cluster_indices.end();++iter_k){
        for (std::vector<int>::const_iterator iter_h = iter_k->indices.begin();iter_h != iter_k->indices.end();iter_h++) {
            onepoint.set_x(incloud->points[*iter_h].x);
            onepoint.set_y(incloud->points[*iter_h].y);
            onepoint.set_z(incloud->points[*iter_h].z);
            clusters[cnt].push_back(onepoint);
        }
        cnt++;
    }
    return true;
}
bool RegCluster::public_clusters(const std::vector<std::vector<common::proto::Point3D>>& clusters,
                     common::proto::Clusters* const public_clusters)
{
    public_clusters->Clear();
    size_t clusters_num = clusters.size();
    common::proto::Cluster* cluster;
    common::proto::Point3D* point3d;
    for (size_t k = 0;k<clusters_num;k++) {
        cluster = public_clusters->add_clusters();
        size_t points_num = clusters[k].size();
        for (size_t h = 0;h<points_num;h++) {
            point3d = cluster->add_points();
            *point3d = clusters[k][h];
        }
    }
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
