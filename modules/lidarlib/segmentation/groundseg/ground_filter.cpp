/**
* ground_filter.cpp
* Author: zhubin
* Created on: 2020-07-02
* Copyright (c) iRotran. All Rights Reserved
*/
#include "ground_filter.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
namespace lidar_algorithm {
namespace points_process {
bool GroundFilter::init()
{
    return true;
}
bool GroundFilter::groundseg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                 pcl::PointIndices::Ptr groundp_indices)
{
    groundp_indices->indices.clear();
    if (!cloud->empty()) {
        sac_seg(cloud,groundp_indices);
    }
    return true;
}
bool GroundFilter::sac_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
             pcl::PointIndices::Ptr groundp_indices)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.15);
    seg.setMaxIterations(100);
    seg.setProbability(0.95);
    seg.setInputCloud (cloud);
    seg.segment (*groundp_indices, *coefficients);
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
