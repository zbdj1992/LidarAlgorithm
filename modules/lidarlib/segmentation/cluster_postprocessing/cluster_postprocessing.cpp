/**
* cluster_postprocessing.cpp
* Author: zhubin
* Created on: 2020-06-28
* Copyright (c) iRotran. All Rights Reserved
*/
#include "cluster_postprocessing.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include <iostream>
namespace lidar_algorithm {
namespace points_process {
const std::string conf_path = "../modules/lidarlib/conf/cluster_postprocessing_conf.yaml";
bool ClusterPostProcessing::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"cluster_merge_thrx",this->_conf_cluster_merge_thrx,0.5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"cluster_merge_thrx conf failed!";
    }else{
        LOG(INFO)<<"cluster_merge_thrx = "<<this->_conf_cluster_merge_thrx;
    }
    readyaml.parse_yaml(parma_node,"cluster_merge_thry",this->_conf_cluster_merge_thry,0.5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"cluster_merge_thry conf failed!";
    }else{
        LOG(INFO)<<"cluster_merge_thry = "<<this->_conf_cluster_merge_thry;
    }
    readyaml.parse_yaml(parma_node,"cluster_merge_thrz",this->_conf_cluster_merge_thrz,1.0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"cluster_merge_thrz conf failed!";
    }else{
        LOG(INFO)<<"cluster_merge_thrz = "<<this->_conf_cluster_merge_thrz;
    }
    readyaml.parse_yaml(parma_node,"use_log_debug",this->_conf_use_log_debug,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_log_debug conf failed!";
    }else{
        LOG(INFO)<<"use_log_debug = "<<this->_conf_use_log_debug;
    }
    return true;
}
bool ClusterPostProcessing::init()
{
    load_conf(conf_path);
    return true;
}
bool ClusterPostProcessing::cluster_merge(const std::vector<std::vector<common::proto::Point3D>>& inclusters,
                   std::vector<std::vector<common::proto::Point3D>>& outclusters)
{
    outclusters.clear();
    int cluster_cnt = 0;
    size_t clusters_num = inclusters.size();
    if (clusters_num <= 0) {
        return false;
    }
    std::vector<common::proto::Point3D> cluster_centrepoint;
    std::vector<int> idvec;
    cluster_centrepoint.resize(clusters_num);
    idvec.resize(clusters_num);
    common::proto::Point3D tmp_centpoint;
    for (size_t k = 0; k< clusters_num; k++) {
        get_centrexyz_clusters(inclusters[k],tmp_centpoint);
        cluster_centrepoint[k] = tmp_centpoint;
        idvec[k] = -1;
    }
    for (size_t k = 0;k < clusters_num;k++){
        if (idvec[k] == -1) {
            idvec[k] = cluster_cnt;
            for (size_t h = 0; h < clusters_num;h++) {
                if (idvec[h] == -1 && fabs(cluster_centrepoint[k].x() - cluster_centrepoint[h].x()) <= _conf_cluster_merge_thrx
                        && abs(cluster_centrepoint[k].y() - cluster_centrepoint[h].y()) <= _conf_cluster_merge_thry
                        && abs(cluster_centrepoint[k].z() - cluster_centrepoint[h].z()) <= _conf_cluster_merge_thrz){
                    idvec[h] = cluster_cnt;
                }
            }
            cluster_cnt++;
        }
    }
    //std::cout<<"cluster_cnt = "<<cluster_cnt<<std::endl;
    outclusters.resize(cluster_cnt);
    for (size_t k = 0;k < clusters_num;k++) {
        int tmpId = idvec[k];
        outclusters[tmpId].insert(outclusters[tmpId].end(),
                                       inclusters[k].begin(),inclusters[k].end());
    }
    return true;
}
bool ClusterPostProcessing::get_centrexyz_clusters(const std::vector<common::proto::Point3D>& cluster,
                            common::proto::Point3D& centrexyz)
{
    common::proto::Point2D boundaryx;
    common::proto::Point2D boundaryy;
    common::proto::Point2D boundaryz;
    get_boundaryxyz_clusters(cluster,boundaryx,boundaryy,boundaryz);
    centrexyz.set_x((boundaryx.x() + boundaryx.y())*0.5);
    centrexyz.set_y((boundaryy.x() + boundaryy.y())*0.5);
    centrexyz.set_z((boundaryz.x() + boundaryz.y())*0.5);
    return true;
}
bool ClusterPostProcessing::get_boundaryxyz_clusters(const std::vector<common::proto::Point3D>& clusters,
                          common::proto::Point2D& boundaryx,
                          common::proto::Point2D& boundaryy,
                          common::proto::Point2D& boundaryz)
{
    size_t vec_size = clusters.size();
    if (vec_size == 0) {
        return false;
    }
    double minx = clusters.front().x();
    double maxx = clusters.front().x();
    double miny = clusters.front().y();
    double maxy = clusters.front().y();
    double minz = clusters.front().z();
    double maxz = clusters.front().z();
    for (size_t k = 0; k < vec_size;k++) {
        minx = (minx < clusters[k].x()) ? minx : clusters[k].x();
        maxx = (maxx > clusters[k].x()) ? maxx : clusters[k].x();
        miny = (miny < clusters[k].y()) ? miny : clusters[k].y();
        maxy = (maxy > clusters[k].y()) ? maxy : clusters[k].y();
        minz = (minz < clusters[k].z()) ? minz : clusters[k].z();
        maxz = (maxz > clusters[k].z()) ? maxz : clusters[k].z();
    }
    boundaryx.set_x(minx);
    boundaryx.set_y(maxx);
    boundaryy.set_x(miny);
    boundaryy.set_y(maxy);
    boundaryz.set_x(minz);
    boundaryz.set_y(maxz);
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
