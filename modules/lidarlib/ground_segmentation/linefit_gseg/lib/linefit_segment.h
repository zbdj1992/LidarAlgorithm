/**
* line_segment.h
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_bin.h"
#include <list>
#include <map>
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
struct SegmentParams
{
    SegmentParams() :
        _n_bins(30),
        _max_slope(1.0),
        _max_error(0.01),
        _long_threshold(2.0),
        _max_long_height(0.1),
        _max_start_height(0.2),
        _sensor_height(0.2),
        _k_margin(0.1){}
    int _n_bins;
    double _max_slope;
    double _max_error;
    double _long_threshold;
    double _max_long_height;
    double _max_start_height;
    double _sensor_height;
    double _k_margin;
};
class Segment {
public:
    typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
    typedef std::pair<double, double> LocalLine;
private:
    std::vector<Bin> bins_;
    std::list<Line> lines_;
    LocalLine fitLocalLine(const std::list<Bin::MinZPoint>& points);
    double getMeanError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);
    double getMaxError(const std::list<Bin::MinZPoint>& points, const LocalLine& line);
    Line localLineToLine(const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points);
    bool td_equal(const double a,const double b);
private:
    int _conf_n_bins;
    double _conf_max_slope;
    double _conf_max_error;
    double _conf_long_threshold;
    double _conf_max_long_height;
    double _conf_max_start_height;
    double _conf_sensor_height;
    double _conf_k_margin;
public:
    Segment();
    bool init(const SegmentParams& params);
    double verticalDistanceToLine(const double& d, const double &z);
    bool fitSegmentLines();
    inline Bin& operator[](const size_t& index) {
        return bins_[index];
    }
    inline std::vector<Bin>::iterator begin() {
        return bins_.begin();
    }
    inline std::vector<Bin>::iterator end() {
        return bins_.end();
    }
    bool getLines(std::list<Line>* lines);
};
}//namespace linefit_gseg
}//namespace points_process
}//namespace lidar_algorithm
#endif /* GROUND_SEGMENTATION_SEGMENT_H_ */
