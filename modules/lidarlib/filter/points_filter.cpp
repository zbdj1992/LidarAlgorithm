/**
* points_filter.cpp
* Author: zhubin
* Created on: 2020-06-27
* Copyright (c) iRotran. All Rights Reserved
*/
#include "points_filter.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include <pcl/filters/voxel_grid.h>
namespace lidar_algorithm {
namespace points_process {
const std::string conf_path = "../modules/lidarlib/conf/points_filter_conf.yaml";
bool PointsFilter::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    readyaml.parse_yaml(parma_node,"vg_size",this->_conf_vg_size,0.02);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"vg_size conf failed!";
    }else{
        LOG(INFO)<<"vg_size = "<<this->_conf_vg_size;
    }
    return true;
}
bool PointsFilter::init()
{
    load_conf(conf_path);
    return true;
}
bool PointsFilter::filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr incloud,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud)
{
    vg_filter(incloud,outcloud);
    return true;
}
bool PointsFilter::vg_filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr incloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud)
{
    if (!incloud->empty()){
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(incloud);
        vg.setLeafSize(_conf_vg_size,_conf_vg_size,_conf_vg_size);
        vg.filter(*outcloud);
        return true;
    }else{
        return false;
    }
}
}//namespace points_process
}//namespace lidar_algorithm
