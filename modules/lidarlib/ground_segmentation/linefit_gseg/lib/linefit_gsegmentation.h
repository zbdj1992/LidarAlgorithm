/**
* linefit_gsegmentation.h
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef GROUND_SEGMENTATION_H_
#define GROUND_SEGMENTATION_H_
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_segment.h"
#include "modules/common/threadpool/threadpool.hpp"
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;
class GroundSegmentation {
public:
    bool init();
    GroundSegmentation();
    bool segment(const PointCloud& cloud, std::vector<int>* segmentation);
public:
    friend void linefit_task(const size_t start,const size_t end,GroundSegmentation* gseg_obj);
    friend void assigncluster_task(const size_t &start_index,const size_t &end_index,
                                   std::vector<int>* segmentation,GroundSegmentation* gseg_obj);
private:
    // Access with segments_[segment][bin].
    std::vector<Segment> segments_;
    // Bin index of every point.
    std::vector<std::pair<int, int> > bin_index_;
    // 2D coordinates (d, z) of every point in its respective segment.
    std::vector<Bin::MinZPoint> segment_coordinates_;
private:
    bool insert_points(const PointCloud& cloud);
    bool linefit();
    /*划分集群*/
    void assign_cluster(std::vector<int>* segmentation);
    /*插入点云*/
    /*得到计算出来的线*/
    void get_lines();
private:
    boost::threadpool::pool _proc_pool;
private:
    bool load_conf(const std::string& file_path);
    int _conf_use_thread_pool;
    // Number of threads.
    int _conf_n_threads;
    // Minimum range of segmentation.
    double _conf_r_min;
    double _conf_r_max;
    double _conf_r_min_square;
    // Maximum range of segmentation.
    double _conf_r_max_square;
    // Number of angular segments.
    int _conf_n_segments;
    // Maximum distance to a ground line to be classified as ground.
    double _conf_max_dist_to_line;
    // How far to search for a line in angular direction [rad].
    double _conf_line_search_angle;
    int _conf_use_debug_log;
    SegmentParams _conf_segment_params;
private:
    bool td_equal(const double a,const double b);
    double _bin_step;
    double _segment_step;
};
}//namespace linefit_gseg
}//namespace points_process
}//namespace lidar_algorithm
#endif // GROUND_SEGMENTATION_H_
