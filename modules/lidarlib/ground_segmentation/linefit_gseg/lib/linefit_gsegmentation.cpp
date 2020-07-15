/**
* linefit_gsegmentation.cpp
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_gsegmentation.h"
#include "modules/common/time/timer.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
#include <iostream>
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
const std::string conf_path = "../modules/lidarlib/conf/linefit_gseg_conf.yaml";
bool GroundSegmentation::load_conf(const std::string& file_path)
{
    using namespace lidar_algorithm::common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    double d_value = 0.0;
    ret = readyaml.parse_yaml(parma_node,"n_threads",this->_conf_n_threads,4);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"n_threads conf failed!";
    }else{
        LOG(INFO)<<"n_threads = "<<this->_conf_n_threads;
    }
    readyaml.parse_yaml(parma_node,"r_min",this->_conf_r_min,0.5);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"r_min conf failed!";
    }else{
        LOG(INFO)<<"r_min = "<<this->_conf_r_min;
    }
    _conf_r_min_square = _conf_r_min*_conf_r_min;
    readyaml.parse_yaml(parma_node,"r_max",this->_conf_r_max,10.0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"r_max conf failed!";
    }else{
        LOG(INFO)<<"r_max = "<<this->_conf_r_max;
    }
    _conf_r_max_square = _conf_r_max*_conf_r_max;
    readyaml.parse_yaml(parma_node,"n_bins",_conf_segment_params._n_bins,1200);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"n_bins conf failed!";
    }else{
        LOG(INFO)<<"n_bins = "<<_conf_segment_params._n_bins;
    }
    readyaml.parse_yaml(parma_node,"n_segments",_conf_n_segments,360);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"n_segments conf failed!";
    }else{
        LOG(INFO)<<"n_segments = "<<_conf_n_segments;
    }
    readyaml.parse_yaml(parma_node,"max_dist_to_line",_conf_max_dist_to_line,0.05);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"max_dist_to_line conf failed!";
    }else{
        LOG(INFO)<<"max_dist_to_line = "<<_conf_max_dist_to_line;
    }
    readyaml.parse_yaml(parma_node,"max_slope",_conf_segment_params._max_slope,0.03);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"max_slope conf failed!";
    }else{
        LOG(INFO)<<"max_slope = "<<_conf_segment_params._max_slope;
    }
    readyaml.parse_yaml(parma_node,"max_fit_error",d_value,0.05);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"max_fit_error conf failed!";
    }else{
        LOG(INFO)<<"max_fit_error = "<<d_value;
    }
    _conf_segment_params._max_error = d_value*d_value;
    readyaml.parse_yaml(parma_node,"long_threshold",_conf_segment_params._long_threshold,1.0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"long_threshold conf failed!";
    }else{
        LOG(INFO)<<"long_threshold = "<<_conf_segment_params._long_threshold;
    }
    readyaml.parse_yaml(parma_node,"max_long_height",_conf_segment_params._max_long_height,0.1);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"max_long_height conf failed!";
    }else{
        LOG(INFO)<<"max_long_height = "<<_conf_segment_params._max_long_height;
    }
    readyaml.parse_yaml(parma_node,"max_start_height",_conf_segment_params._max_start_height,0.2);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"max_start_height conf failed!";
    }else{
        LOG(INFO)<<"max_start_height = "<<_conf_segment_params._max_start_height;
    }
    readyaml.parse_yaml(parma_node,"sensor_height",_conf_segment_params._sensor_height,0.24);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"sensor_height conf failed!";
    }else{
        LOG(INFO)<<"sensor_height = "<<_conf_segment_params._sensor_height;
    }
    readyaml.parse_yaml(parma_node,"line_search_angle",_conf_line_search_angle,0.1);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"line_search_angle conf failed!";
    }else{
        LOG(INFO)<<"line_search_angle = "<<_conf_line_search_angle;
    }
    readyaml.parse_yaml(parma_node,"k_margin",_conf_segment_params._k_margin,0.1);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"k_margin conf failed!";
    }else{
        LOG(INFO)<<"k_margin = "<<_conf_segment_params._k_margin;
    }
    return true;
}
bool GroundSegmentation::init()
{
    //load conf
    load_conf(conf_path);
    //segment init
    segments_.resize(_conf_n_segments);
    for (int k = 0;k<_conf_n_segments;++k) {
        segments_[k].init(_conf_segment_params);
    }
    _proc_pool = _conf_n_threads;
    _segment_step = M_PI/_conf_n_segments;
    _bin_step = (_conf_r_max - _conf_r_min)/_conf_segment_params._n_bins;
    return true;
}
GroundSegmentation::GroundSegmentation()
{

}
bool GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
    if (cloud.empty()) {
        return false;
    }
    LOG(WARNING)<<" cloud size: "<<cloud.size();
    common::time::Timer proc_insert_timer;
    segmentation->resize(cloud.size(), 0);
    bin_index_.resize(cloud.size());
    segment_coordinates_.resize(cloud.size());
    insert_points(cloud);
    LOG(ERROR)<< "insert points Took "<< proc_insert_timer.elapsed()<< "ms\n";
    common::time::Timer proc_get_lines_timer;
//    linefit();
    linefit_task(0,_conf_n_segments - 1,this);
    LOG(ERROR)<< "get lines Took "<< proc_get_lines_timer.elapsed()<< "ms\n";
    common::time::Timer proc_assign_cluster_timer;
    assigncluster_task(0,cloud.size()-1,segmentation,this);
    //assign_cluster(segmentation);
    LOG(ERROR)<< "assign cluster Took "<< proc_assign_cluster_timer.elapsed()<< "ms\n";
    return true;
}
void linefit_task(const size_t start,const size_t end,GroundSegmentation* gseg_obj)
{
    for (size_t k = start; k < end; ++k) {
        //LOG(WARNING)<<" segment k: "<<k;
        gseg_obj->segments_[k].fitSegmentLines();
    }
}
bool GroundSegmentation::linefit() {
    std::vector<size_t> p_vec(_conf_n_threads+1);
    for (int k = 0;k<_conf_n_threads;++k) {
        p_vec[k] = _conf_n_segments/_conf_n_threads * k;
    }
    p_vec[_conf_n_threads] = _conf_n_segments - 1;
    for (int k = 0;k<_conf_n_threads;++k) {
        _proc_pool.schedule(boost::bind(linefit_task,p_vec[k],p_vec[k+1],this));
    }
    _proc_pool.wait();
    return true;
}
void assigncluster_task(const size_t &start_index,const size_t &end_index,
                               std::vector<int>* segmentation,GroundSegmentation* gseg_obj)
{
    Bin::MinZPoint point_2d;
    int segment_index = 0;
    int index_1 = 0;
    int index_2 = 0;
    double dist = 0;
    double dist_1 = 0;
    double dist_2 = 0;
    int steps = 1;
    for (size_t i(start_index); i != end_index; ++i) {
          point_2d = gseg_obj->segment_coordinates_[i];
        if (point_2d.z > 0.1) {
            continue;
        }
        segment_index = gseg_obj->bin_index_[i].first;
        if (segment_index >= 0) {
            dist = gseg_obj->segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
            // Search neighboring segments.
            steps = 1;
            while ( (dist < 0) && (steps * gseg_obj->_segment_step < gseg_obj->_conf_line_search_angle)) {
                // Fix indices that are out of bounds.
                index_1 = segment_index + steps;
                while(index_1 >= gseg_obj->_conf_n_segments) {
                    index_1 -= gseg_obj->_conf_n_segments;
                }
                index_2 = segment_index - steps;
                while(index_2 < 0) {
                    index_2 += gseg_obj->_conf_n_segments;
                }
                // Get distance to neighboring lines.
                //std::cout<<" segment_step: "<<gseg_obj->_segment_step<<" segment_index: "<<segment_index<<" steps: "<<steps<<" index_1: "<<index_1<<" index_2: "<<index_2<<std::endl;
                dist_1 = gseg_obj->segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
                dist_2 = gseg_obj->segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
                // Select larger distance if both segments return a valid distance.
                if (dist_1 > dist) {
                    dist = dist_1;
                }
                if (dist_2 > dist) {
                    dist = dist_2;
                }
                ++steps;
            }
            if (dist < gseg_obj->_conf_max_dist_to_line && (dist >= 0.0)) {
                segmentation->at(i) = 1;
            }
        }
    }
}
void GroundSegmentation::assign_cluster(std::vector<int>* segmentation) {
    size_t cloud_size = segmentation->size();
    std::vector<size_t> p_vec(_conf_n_threads+1);
    //std::cout<<" assign p[k]: "<<std::endl;
    for (int k = 0;k<_conf_n_threads;++k) {
        p_vec[k] = cloud_size/_conf_n_threads * k;
        //std::cout<<" k: "<<k<<" p: "<<p_vec[k]<<std::endl;
    }
    p_vec[_conf_n_threads] = cloud_size - 1;
    //std::cout<<" k: "<< _conf_n_threads<<" p: "<<p_vec[_conf_n_threads]<<std::endl;
    for (int k = 0;k<_conf_n_threads;++k) {
        _proc_pool.schedule(boost::bind(assigncluster_task,p_vec[k],p_vec[k+1],segmentation,this));
    }
    _proc_pool.wait();
}
bool GroundSegmentation::insert_points(const PointCloud& cloud) {
    size_t points_num = cloud.size();
    if (points_num == 0) {
        return false;
    }
    pcl::PointXYZ point;
    double range_square = 0;
    double range = 0;
    double angle = 0;
    size_t bin_index = 0;
    size_t segment_index = 0;
    for (size_t k = 0; k < points_num; ++k) {
        point = cloud[k];
        if (point.z > 0.2) {
            continue;
        }
        range_square = point.x * point.x + point.y * point.y;
        range = sqrt(range_square);
        if ((range_square < _conf_r_max_square) && (range_square > _conf_r_min_square)) {
            angle = std::atan2(point.y, point.x); //[-pi,pi]
            bin_index = (range - _conf_r_min) / _bin_step;
//            segment_index = (angle + M_PI) / _segment_step;
            segment_index = (M_PI - angle) / _segment_step;
            //LOG(WARNING)<<" angle: "<<angle<<" segment_index: "<<segment_index<<" bin_index: "<<bin_index;
            segments_[segment_index][bin_index].addPoint(range, point.z);
            bin_index_[k] = std::make_pair(segment_index, bin_index);
            //segment_coordinates_[k] = Bin::MinZPoint(range, point.z);
        }else {
            bin_index_[k] = std::make_pair<int, int>(-1, -1);
            //segment_coordinates_[k] = Bin::MinZPoint(std::numeric_limits<double>::max(), point.z);
        }
        segment_coordinates_[k] = Bin::MinZPoint(range, point.z);
    }
    return true;
}
bool GroundSegmentation::td_equal(const double a,const double b)
{
    bool is_equal = false;
    if (((a-b)>-0.000001)&&((a-b)<0.000001)) {
        is_equal = true;
    }
    return is_equal;
}
}//namespace linefit_gseg
}//namespace points_process
}//namespace lidar_algorithm
