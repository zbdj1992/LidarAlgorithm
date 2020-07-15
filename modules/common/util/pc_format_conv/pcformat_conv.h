/**
* pcformat_conv.h
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef PCFORMAT_CONV_H
#define PCFORMAT_CONV_H
#include "modules/common/proto/geometry.pb.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
namespace lidar_algorithm {
namespace common {
namespace util {
class PCFormatConv
{
public:
    PCFormatConv();
    ~PCFormatConv();
public:
    bool pcl_proto(const pcl::PointCloud<pcl::PointXYZI>& pcl_format,
                   common::proto::PointCloud& proto_format);
    bool proto_pcl(const common::proto::PointCloud& proto_format,
                   pcl::PointCloud<pcl::PointXYZI>& pcl_format);
    bool proto_pcl(const common::proto::PointCloud& proto_format,
                   pcl::PointCloud<pcl::PointXYZ>& pcl_format);
    bool proto_pcl(const common::proto::Cluster& proto_format,
                   pcl::PointCloud<pcl::PointXYZ>& pcl_format);
};
}//namespace util
}//namespace common
}//namespace lidar_algorithm
#endif // PCFORMAT_CONV_H
