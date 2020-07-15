#include "eu_cluster.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include "modules/lidarlib/segmentation/cluster_postprocessing/cluster_postprocessing.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <eigen3/Eigen/Core>
namespace lidar_algorithm {
namespace points_process {
const std::string conf_path = "../modules/lidarlib/conf/eu_cluster_conf.yaml";
bool EuCluster::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"ec_cluster_tolerence",this->_conf_ec_cluster_tolerence,0.1);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"ec_cluster_tolerence conf failed!";
    }else{
        LOG(INFO)<<"ec_cluster_tolerence = "<<this->_conf_ec_cluster_tolerence;
    }
    readyaml.parse_yaml(parma_node,"ec_cluster_min_size",this->_conf_ec_cluster_min_size,5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"ec_cluster_min_size conf failed!";
    }else{
        LOG(INFO)<<"ec_cluster_min_size = "<<this->_conf_ec_cluster_min_size;
    }
    readyaml.parse_yaml(parma_node,"ec_cluster_max_size",this->_conf_ec_cluster_max_size,5000);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"ec_cluster_max_size conf failed!";
    }else{
        LOG(INFO)<<"ec_cluster_max_size = "<<this->_conf_ec_cluster_max_size;
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
bool EuCluster::init()
{
    //load conf
    load_conf(conf_path);
    //load post processing method
    _postprocessing.reset(new ClusterPostProcessing);
    _postprocessing->init();
    return true;
}
bool EuCluster::cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                 common::proto::Clusters* const clusters)
{
    std::vector<std::vector<common::proto::Point3D>> origin_clusters;
    std::vector<std::vector<common::proto::Point3D>> merge_clusters;
    std::vector<pcl::PointIndices> cluster_indices;
    eu_cluster(cloud,cluster_indices);
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
bool EuCluster::eu_cluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
            std::vector<pcl::PointIndices>& cluster_indices)
{
    cluster_indices.clear();
    size_t cloud_num = incloud->size();
    if (cloud_num == 0) {
        return false;
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(incloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(_conf_ec_cluster_tolerence); //zb 0.6->0.06
    ec.setMinClusterSize(_conf_ec_cluster_min_size);     //zb 1->30
    ec.setMaxClusterSize(_conf_ec_cluster_max_size);  //zb 2000->400
    ec.setSearchMethod(tree);
    ec.setInputCloud(incloud);
    ec.extract(cluster_indices);
    return true;
}
bool EuCluster::get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
                  std::vector<std::vector<common::proto::Point3D>>& clusters)
{
    int clusters_num = cluster_indices.size();
    clusters.resize(clusters_num);
    int cnt = 0;
    common::proto::Point3D onepoint;
    for (std::vector<pcl::PointIndices>::const_iterator iter_k = cluster_indices.begin();iter_k != cluster_indices.end();++iter_k){
        for (std::vector<int>::const_iterator iter_h = iter_k->indices.begin();iter_h != iter_k->indices.end();++iter_h) {
            onepoint.set_x(incloud->points[*iter_h].x);
            onepoint.set_y(incloud->points[*iter_h].y);
            onepoint.set_z(incloud->points[*iter_h].z);
            clusters[cnt].push_back(onepoint);
        }
        cnt++;
    }
    return true;
}
bool EuCluster::get_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr incloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
                  common::proto::Clusters* const clusters)
{
    clusters->Clear();
//    size_t num_clusters = cluster_indices.size();
    common::proto::Cluster* cluster;
    common::proto::Point3D* point3d;
    for (std::vector<pcl::PointIndices>::const_iterator iter_k = cluster_indices.begin();iter_k != cluster_indices.end();++iter_k){
        cluster = clusters->add_clusters();
        for (std::vector<int>::const_iterator iter_h = iter_k->indices.begin();iter_h != iter_k->indices.end();++iter_h) {
            point3d = cluster->add_points();
            point3d->set_x(incloud->points[*iter_h].x);
            point3d->set_y(incloud->points[*iter_h].y);
            point3d->set_z(incloud->points[*iter_h].z);
        }
    }
    return true;
}
bool EuCluster::public_clusters(const std::vector<std::vector<common::proto::Point3D>>& clusters,
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
