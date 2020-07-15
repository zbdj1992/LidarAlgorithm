/**
* pcformat_conv.cpp
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#include "pcformat_conv.h"
namespace lidar_algorithm {
namespace common {
namespace util {
PCFormatConv::PCFormatConv()
{

}
PCFormatConv::~PCFormatConv()
{

}
bool PCFormatConv::pcl_proto(const pcl::PointCloud<pcl::PointXYZI>& pcl_format,
               common::proto::PointCloud& proto_format)
{
    size_t points_num = pcl_format.size();
    common::proto::PointXYZIL *proto_point;
    proto_format.clear_points();
    for (size_t k = 0;k<points_num;k++) {
        proto_point = proto_format.add_points();
        proto_point->set_x(pcl_format.points[k].x);
        proto_point->set_y(pcl_format.points[k].y);
        proto_point->set_z(pcl_format.points[k].z);
        proto_point->set_intensity(pcl_format.points[k].intensity);
        proto_point->set_label(0);
    }
    return true;
}
bool PCFormatConv::proto_pcl(const common::proto::PointCloud& proto_format,
               pcl::PointCloud<pcl::PointXYZI>& pcl_format)
{
    size_t points_num = proto_format.points_size();
    pcl_format.clear();
    pcl::PointXYZI one_point;
    for (size_t k = 0;k<points_num;k++) {
        one_point.x = proto_format.points(k).x();
        one_point.y = proto_format.points(k).y();
        one_point.z = proto_format.points(k).z();
        one_point.intensity = proto_format.points(k).intensity();
        pcl_format.points.push_back(one_point);
    }
    pcl_format.width = pcl_format.points.size();
    pcl_format.height = 1;
    return true;
}
bool PCFormatConv::proto_pcl(const common::proto::PointCloud& proto_format,
               pcl::PointCloud<pcl::PointXYZ>& pcl_format)
{
    size_t points_num = proto_format.points_size();
    pcl_format.clear();
    pcl::PointXYZ one_point;
    for (size_t k = 0;k<points_num;k++) {
        one_point.x = proto_format.points(k).x();
        one_point.y = proto_format.points(k).y();
        one_point.z = proto_format.points(k).z();
        pcl_format.points.push_back(one_point);
    }
    pcl_format.width = pcl_format.points.size();
    pcl_format.height = 1;
    return true;
}
bool PCFormatConv::proto_pcl(const common::proto::Cluster& proto_format,
               pcl::PointCloud<pcl::PointXYZ>& pcl_format)
{
    size_t points_num = proto_format.points_size();
    pcl_format.clear();
    pcl::PointXYZ one_point;
    for (size_t k = 0;k<points_num;k++) {
        one_point.x = proto_format.points(k).x();
        one_point.y = proto_format.points(k).y();
        one_point.z = proto_format.points(k).z();
        pcl_format.points.push_back(one_point);
    }
    pcl_format.width = pcl_format.points.size();
    pcl_format.height = 1;
    return true;
}
}//namespace util
}//namespace common
}//namespace lidar_algorithm
