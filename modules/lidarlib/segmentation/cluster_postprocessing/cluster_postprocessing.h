/**
* cluster_postprocessing.h
* Author: zhubin
* Created on: 2020-06-28
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef CLUSTER_POSTPROCESSING_H
#define CLUSTER_POSTPROCESSING_H
#include "modules/lidarlib/interface/base_post_processing.h"
namespace lidar_algorithm {
namespace points_process {
class ClusterPostProcessing : public BasePostProcessing
{
public:
    ClusterPostProcessing(){}
    virtual ~ClusterPostProcessing(){}
public:
    bool init() override;
    bool cluster_merge(const std::vector<std::vector<common::proto::Point3D>>& inclusters,
                       std::vector<std::vector<common::proto::Point3D>>& outclusters) override;
private:
    bool get_centrexyz_clusters(const std::vector<common::proto::Point3D>& cluster,
                                common::proto::Point3D& centre);
    bool get_boundaryxyz_clusters(const std::vector<common::proto::Point3D>& clusters,
                              common::proto::Point2D& boundaryx,
                              common::proto::Point2D& boundaryy,
                              common::proto::Point2D& boundaryz);
private:
    bool load_conf(const std::string& file_path);
    double _conf_cluster_merge_thrx;
    double _conf_cluster_merge_thry;
    double _conf_cluster_merge_thrz;
    int _conf_use_log_debug;
};
}//namespace points_process
}//namespace lidar_algorithm
#endif // CLUSTER_POSTPROCESSING_H
