/**
* points_process.cpp
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#include "points_process.h"
#include "modules/framework/conf/parameter.h"
#include "modules/common/time/timer.h"
#include "modules/common/time/timestamp.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include "modules/lidarlib/boxbuilder/box_builder.h"
#include "modules/lidarlib/segmentation/eucluster/eu_cluster.h"
#include "modules/lidarlib/segmentation/regcluster/reg_cluster.h"
#include "modules/lidarlib/segmentation/groundseg/ground_filter.h"
#include "modules/lidarlib/ground_segmentation/linefit_gseg/linefit_gseg.h"
#include "modules/lidarlib/filter/points_filter.h"
#include <pcl/filters/extract_indices.h>
namespace lidar_algorithm {
namespace points_process {
const std::string conf_path = "../modules/lidarlib/conf/lidarlib_conf.yaml";
PointsProcess::PointsProcess()
{

}
bool PointsProcess::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"cluster_algorithm_elect",this->_conf_cluster_algorithm_elect,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"cluster_algorithm_elect conf failed!";
    }else{
        LOG(INFO)<<"cluster_algorithm_elect = "<<this->_conf_cluster_algorithm_elect;
    }
    ret = readyaml.parse_yaml(parma_node,"use_ground_points_filter",this->_conf_use_ground_points_filter,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_ground_points_filter conf failed!";
    }else{
        LOG(INFO)<<"use_ground_points_filter = "<<this->_conf_use_ground_points_filter;
    }
    ret = readyaml.parse_yaml(parma_node,"use_vg_filter",this->_conf_use_vg_filter,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_vg_filter conf failed!";
    }else{
        LOG(INFO)<<"use_vg_filter = "<<this->_conf_use_vg_filter;
    }
    return true;
}
bool PointsProcess::init()
{
    //load conf
    load_conf(conf_path);
    //load boxbuilder method
    _boxbuilder.reset(new BoxBuilder);
    _boxbuilder->init();
    //load eucluster method
    _eucluster.reset(new EuCluster);
    _eucluster->init();
    //load regcluster method
    _regcluster.reset(new RegCluster);
    _regcluster->init();
    //load groundfilter method
    _groundfilter.reset(new GroundFilter);
    _groundfilter->init();
    //load linefilt groundseg method
    _lfgseg.reset(new LinefitGSeg);
    _lfgseg->init();
    //load vgfilter method
    _vgfilter.reset(new PointsFilter);
    _vgfilter->init();
    return true;
}
bool PointsProcess::process()
{
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    _lidarlib_thread = std::thread(&PointsProcess::thread_proc,this);
    if (conf_obj->_lidarlib_thread_join){
        _lidarlib_thread.join();
    }else{
        _lidarlib_thread.detach();
    }
    return true;
}
bool PointsProcess::thread_proc(void)
{
    LOG(INFO)<<"lidarlib process thread ready! ";
    std::cout <<"lidarlib process thread ready!"<<std::endl;
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    framework::data::Data *data_obj = framework::data::Data::get_obj();
    common::proto::PointCloud proto_cloud_data;
    common::proto::PointCloud vgfilter_proto_cloud_data;
    pcl::PointCloud<pcl::PointXYZI>::Ptr origin_pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr vgfilter_pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> glabels;
    common::proto::Clusters clusters;
    common::proto::AABB3Ds aabb3ds;
    common::proto::OBB3Ds obb3ds;
    framework::data::proto::LidarLibData lidarlib_data;
    long long cnt = 0;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    while(true){
        common::time::Timer proc_timer;
        proto_cloud_data = data_obj->get_one_pointcloud();
        _pcformatconv.proto_pcl(proto_cloud_data,*origin_pcl_cloud);
        if (_conf_use_vg_filter) {
            _vgfilter->filter(origin_pcl_cloud,vgfilter_pcl_cloud);
            if (_conf_use_ground_points_filter) {
                //ground points filter
                ground_points_seg(vgfilter_pcl_cloud,&glabels);
                //update pointcloud label
                update_pointcloud_label(glabels,vgfilter_pcl_cloud,filter_groundp_cloud,vgfilter_proto_cloud_data);
                //cluster
                cluster_proc(filter_groundp_cloud,&clusters);
            }else{
                //cluster
                cluster_proc(vgfilter_pcl_cloud,&clusters);
            }
//            LOG(WARNING)<<"origin cloud size: "<<origin_pcl_cloud->size()
//                       <<" vg cloud size: "<<vgfilter_pcl_cloud->size()
//                      <<" glabels size: "<<glabels.size()
//                     <<" proto cloud size: "<<proto_cloud_data.points_size()
//                    <<" vg proto cloud size: "<<vgfilter_proto_cloud_data.points_size()
//                   <<" filter ground cloud size: "<<filter_groundp_cloud->size();
        }else{
            if (_conf_use_ground_points_filter) {
                //ground points filter
                ground_points_seg(origin_pcl_cloud,&glabels);
                //update pointcloud label
                update_pointcloud_label(glabels,proto_cloud_data,filter_groundp_cloud);
                //cluster
                cluster_proc(filter_groundp_cloud,&clusters);
            }else{
                //cluster
                cluster_proc(origin_pcl_cloud,&clusters);
            }
        }
        //boundingbox
        _boxbuilder->boxbuild(clusters,&aabb3ds,&obb3ds);
        //update
        if (_conf_use_vg_filter) {
            update_public(clusters,aabb3ds,obb3ds,vgfilter_proto_cloud_data,lidarlib_data);
        }else{
            update_public(clusters,aabb3ds,obb3ds,proto_cloud_data,lidarlib_data);
        }
        //public
        data_obj->set_one_lidarlib_data(lidarlib_data);
        cnt++;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(conf_obj->_lidarlib_period));
        if (conf_obj->_use_data_log) {
            if (cnt%conf_obj->_log_save_freq == 0){
                LOG(INFO)<<"LidarLib Modules: "
                        <<" cnt= "<<cnt
                       << " process elapsed= "<<proc_timer.elapsed() << " ms";
            }
        }
    }
    return true;
}
bool PointsProcess::update_public(const common::proto::Clusters& clusters,
                 const common::proto::AABB3Ds& aabb3ds,
                 const common::proto::OBB3Ds& obb3ds,
                 const common::proto::PointCloud& proto_pointcloud,
                 framework::data::proto::LidarLibData& lidarlib_data)
{
    lidarlib_data.Clear();
    //timestamp
    long long timestamp = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_milliseconds);
    lidarlib_data.set_timestamp(timestamp);
    //clusters
    common::proto::Clusters* pclusters = lidarlib_data.mutable_clusters();
    *pclusters = clusters;
    //aabb3ds
    common::proto::AABB3Ds* paabbs = lidarlib_data.mutable_aabb3ds();
    *paabbs = aabb3ds;
    //obb3ds
    common::proto::OBB3Ds* pobbs = lidarlib_data.mutable_obb3ds();
    *pobbs = obb3ds;
    //pointcloud
    common::proto::PointCloud* ppointcloud = lidarlib_data.mutable_point_cloud();
    *ppointcloud = proto_pointcloud;
    return true;
}
bool PointsProcess::cluster_proc(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_xyzi,
                  common::proto::Clusters* const clusters)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyzi, *cloud_xyz);
    switch (_conf_cluster_algorithm_elect) {
    case 0:
        _eucluster->cluster(cloud_xyz,clusters);
        break;
    case 1:
        _regcluster->cluster(cloud_xyz,clusters);
        break;
    default:
        _eucluster->cluster(cloud_xyz,clusters);
        break;
    }
    return true;
}
bool PointsProcess::ground_points_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                       pcl::PointIndices::Ptr groundp_indices,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud)
{
    if (_conf_use_ground_points_filter) {
        //ground points filter
        _groundfilter->groundseg(cloud,groundp_indices);
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(groundp_indices);
        extract.setNegative (true);//extract not ground points
        extract.filter(*filter_groundp_cloud);
    }else{
        pcl::copyPointCloud(*cloud,*filter_groundp_cloud);
    }
    return true;
}
bool PointsProcess::ground_points_seg(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                       std::vector<int>* const glabels)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);
    _lfgseg->gseg(cloud_xyz,glabels);
    return true;
}
bool PointsProcess::update_pointcloud_label(const pcl::PointIndices::ConstPtr groundp_indices,
                             common::proto::PointCloud& proto_cloud_data)
{
    long long time_stamp = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_milliseconds);
    proto_cloud_data.set_timestamp(time_stamp);
    size_t points_num = groundp_indices->indices.size();
    for (size_t k = 0;k<points_num;++k) {
        proto_cloud_data.mutable_points(groundp_indices->indices[k])->set_label(1);
    }
    return true;
}
bool PointsProcess::update_pointcloud_label(const std::vector<int>& glabels,
                             common::proto::PointCloud& proto_cloud_data,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud)
{
    long long time_stamp = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_milliseconds);
    filter_groundp_cloud->clear();
    proto_cloud_data.set_timestamp(time_stamp);
    size_t points_num = glabels.size();
    pcl::PointXYZI onepoint;
    int obspoint_cnt = 0;
    for (size_t k = 0;k<points_num;++k) {
        if (glabels[k] == 1) {
            proto_cloud_data.mutable_points(k)->set_label(1);
        }else{
            onepoint.x = proto_cloud_data.points(k).x();
            onepoint.y = proto_cloud_data.points(k).y();
            onepoint.z = proto_cloud_data.points(k).z();
            onepoint.intensity = proto_cloud_data.points(k).intensity();
            filter_groundp_cloud->points.push_back(onepoint);
            obspoint_cnt++;
        }
    }
    filter_groundp_cloud->width = obspoint_cnt;
    filter_groundp_cloud->height = 1;
    return true;
}
bool PointsProcess::update_pointcloud_label(const std::vector<int>& glabels,
                             const pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_cloud_data,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr filter_groundp_cloud,
                             common::proto::PointCloud& proto_cloud_data)
{
    long long time_stamp = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_milliseconds);
    proto_cloud_data.Clear();
    filter_groundp_cloud->clear();
    proto_cloud_data.set_timestamp(time_stamp);
    size_t points_num = glabels.size();
    common::proto::PointXYZIL *pointxyzl;
    int obspoint_cnt = 0;
    for (size_t k = 0;k<points_num;++k) {
        pointxyzl = proto_cloud_data.add_points();
        pointxyzl->set_x(pcl_cloud_data->points[k].x);
        pointxyzl->set_y(pcl_cloud_data->points[k].y);
        pointxyzl->set_z(pcl_cloud_data->points[k].z);
        pointxyzl->set_intensity(pcl_cloud_data->points[k].intensity);
        if (glabels[k] == 1) {
            pointxyzl->set_label(1);
        }else{
            pointxyzl->set_label(0);
            filter_groundp_cloud->points.push_back(pcl_cloud_data->points[k]);
            obspoint_cnt++;
        }
    }
    filter_groundp_cloud->width = obspoint_cnt;
    filter_groundp_cloud->height = 1;
    return true;
}
}//namespace points_process
}//namespace lidar_algorithm
