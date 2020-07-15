/**
* pcl_show3d.h
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef PCL_SHOW3D_H
#define PCL_SHOW3D_H
#include "modules/framework/data/data.h"
#include "modules/common/util/pc_format_conv/pcformat_conv.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <vector>
#include <string>
namespace lidar_algorithm {
namespace pcl_show {
class PclShow3D
{
public:
    PclShow3D();
    ~PclShow3D();
public:
    bool init();
    bool process();
private:
    std::thread _pclshow_thread;
    bool thread_proc(void);
private:
    bool load_conf(const std::string& file_path);
    int _conf_cloud_show;
    int _conf_aabb_show;
    int _conf_obb_show;
private:
    pcl::visualization::PCLVisualizer::Ptr _viewer_ptr;
    pcl::visualization::PCLVisualizer::Ptr init_pclvis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_cloud);
    bool update_pclvis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_cloud);
    bool add_cube(const framework::data::proto::LidarLibData& lidarlib_data);
    bool spilt_cloud(const framework::data::proto::LidarLibData& lidarlib_data,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
private:
    common::util::PCFormatConv _pcformatconv;
};
}//namespace pcl_show
}//namespace lidar_algorithm
#endif // PCL_SHOW3D_H
