/**
* box_builder.cpp
* Author: zhubin
* Created on: 2020-06-28
* Copyright (c) iRotran. All Rights Reserved
*/
#include "box_builder.h"
#include <pcl/features/moment_of_inertia_estimation.h>
namespace lidar_algorithm {
namespace points_process {
bool BoxBuilder::init()
{
    return true;
}
bool BoxBuilder::boxbuild(const common::proto::Clusters& clusters,
                 common::proto::AABB3Ds* const aabb3ds,
                      common::proto::OBB3Ds* const obb3ds)
{
    aabb3ds->Clear();
    obb3ds->Clear();
    common::proto::AABB3D *aabb3d;
    common::proto::OBB3D *obb3d;
    int clusters_num = clusters.clusters_size();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (int k = 0;k<clusters_num;++k) {
        aabb3d = aabb3ds->add_aabb3ds();
        obb3d = obb3ds->add_obb3ds();
        _pcformatconv.proto_pcl(clusters.clusters(k),*cloud_cluster);
        get_box(cloud_cluster,aabb3d,obb3d);
    }
    return true;
}
bool BoxBuilder::get_box(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster_cloud,
             common::proto::AABB3D* const aabb3d,
             common::proto::OBB3D* const obb3d)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.setInputCloud (cluster_cloud);
    feature_extractor.compute ();
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    //AABB
    aabb3d->mutable_min_point()->set_x(min_point_AABB.x);
    aabb3d->mutable_min_point()->set_y(min_point_AABB.y);
    aabb3d->mutable_min_point()->set_z(min_point_AABB.z);
    aabb3d->mutable_max_point()->set_x(max_point_AABB.x);
    aabb3d->mutable_max_point()->set_y(max_point_AABB.y);
    aabb3d->mutable_max_point()->set_z(max_point_AABB.z);
    //OBB
    //min point
    obb3d->mutable_min_point()->set_x(min_point_OBB.x);
    obb3d->mutable_min_point()->set_y(min_point_OBB.y);
    obb3d->mutable_min_point()->set_z(min_point_OBB.z);
    //max point
    obb3d->mutable_max_point()->set_x(max_point_OBB.x);
    obb3d->mutable_max_point()->set_y(max_point_OBB.y);
    obb3d->mutable_max_point()->set_z(max_point_OBB.z);
    //position
    obb3d->mutable_position()->set_x(position_OBB.x);
    obb3d->mutable_position()->set_y(position_OBB.y);
    obb3d->mutable_position()->set_z(position_OBB.z);
    //Quaternion
    obb3d->mutable_quat()->set_w(quat.w());
    obb3d->mutable_quat()->set_x(quat.x());
    obb3d->mutable_quat()->set_y(quat.y());
    obb3d->mutable_quat()->set_z(quat.z());
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
