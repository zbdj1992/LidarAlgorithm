/**
* points_process.h
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef POINTSPROCESS_H
#define POINTSPROCESS_H
#include "modules/framework/data/data.h"
#include "modules/common/util/pc_format_conv/pcformat_conv.h"
#include "modules/lidarlib/interface/base_cluster.h"
#include "modules/lidarlib/interface/base_box_builder.h"
#include "modules/lidarlib/interface/base_ground_seg.h"
#include "modules/lidarlib/interface/base_linefilt_gseg.h"
#include "modules/lidarlib/interface/base_filter.h"
#include <thread>
#include <vector>
#include <string>
namespace lidar_algorithm {
namespace points_process {
class PointsProcess
{
public:
    PointsProcess();
public:
    bool init();
    bool process();
private:
    std::thread _lidarlib_thread;
    bool thread_proc(void);
private:
    bool load_conf(const std::string& file_path);
    int _conf_cluster_algorithm_elect;//0:eucluster 1:regcluster
    int _conf_use_ground_points_filter;
    int _conf_use_vg_filter;
private:
    common::util::PCFormatConv _pcformatconv;
private:
    bool update_public(const common::proto::Clusters& clusters,
                       const common::proto::AABB3Ds& aabb3ds,
                       const common::proto::OBB3Ds& obb3ds,
                       const common::proto::PointCloud& proto_pointcloud,
                       framework::data::proto::LidarLibData& lidarlib_data);
    bool cluster_proc(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_xyzi,
                      common::proto::Clusters* const clusters);
    bool ground_points_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                           pcl::PointIndices::Ptr groundp_indices,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud);
    bool ground_points_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                           std::vector<int>* const glabels);
    bool update_pointcloud_label(const pcl::PointIndices::ConstPtr groundp_indices,
                                 common::proto::PointCloud& proto_cloud_data);
    bool update_pointcloud_label(const std::vector<int>& glabels,
                                 common::proto::PointCloud& proto_cloud_data,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud);
    bool update_pointcloud_label(const std::vector<int>& glabels,
                                 const pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_cloud_data,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud,
                                 common::proto::PointCloud& proto_cloud_data);
private:
    std::unique_ptr<BaseBoxBuilder> _boxbuilder;
    std::unique_ptr<BaseCluster> _eucluster;
    std::unique_ptr<BaseCluster> _regcluster;
    std::unique_ptr<BaseGroundFilter> _groundfilter;
    std::unique_ptr<BaseLinefitGSeg> _lfgseg;
    std::unique_ptr<BaseFilter> _vgfilter;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // POINTSPROCESS_H
