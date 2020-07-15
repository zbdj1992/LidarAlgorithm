/**
* lidar_driver.h
* Author: zhubin
* Created on: 2020-06-17
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/util/pc_format_conv/pcformat_conv.h"
#include "modules/common/util/readtext/read_text.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <vector>
#include <string>
namespace lidar_algorithm {
namespace drivers {
namespace lidar {
class LidarDriver
{
public:
    LidarDriver();
    ~LidarDriver();
public:
    bool init(common::proto::PointCloud* const lidar_cloud);
private:
    std::thread _lidar_thread;
    bool thread_proc(common::proto::PointCloud* const lidar_cloud);
private:
    bool load_conf(const std::string& conf_path);
    bool load_pcd(const std::string& file_path);
    bool pass_filter();
    bool pubilc_data(common::proto::PointCloud* const lidar_cloud);
private:
    pcl::PointCloud<pcl::PointXYZI> _pcl_pointcloud;
    pcl::PointCloud<pcl::PointXYZI> _pcl_pointcloud_filtered;
    common::proto::PointCloud _proto_pointcloud;
    std::vector<std::string> _pcdpath_vec;
    int _pcd_nums;
private:
    std::string _conf_load_pcd_path;
    int _conf_data_cyclical_pattern;
    int _conf_use_passthrough_filter;
    float _conf_lidar_roi_x_left;
    float _conf_lidar_roi_x_right;
    float _conf_lidar_roi_y_front;
    float _conf_lidar_roi_y_after;
    float _conf_lidar_roi_z_down;
    float _conf_lidar_roi_z_up;
    float _conf_lidar_roi_intensity_min;
    float _conf_lidar_roi_intensity_max;
private:
    common::util::PCFormatConv _pcformatconv;
    common::util::ReadText _readtext;
};

}//namespace lidar
}//namespace drivers
}//namespace lidar_algorithm
#endif // LIDARDRIVER_H
