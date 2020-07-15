/**
* line_segment.cpp
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#include "modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_segment.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/time/timer.h"
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
Segment::Segment()
{

}
bool Segment::init(const SegmentParams& params)
{
    _conf_n_bins = params._n_bins;
    _conf_max_slope = params._max_slope;
    _conf_max_error = params._max_error;
    _conf_long_threshold = params._long_threshold;
    _conf_max_long_height = params._max_long_height;
    _conf_max_start_height = params._max_start_height;
    _conf_sensor_height = params._sensor_height;
    _conf_k_margin = params._k_margin;
    bins_.resize(_conf_n_bins);
    LOG(INFO)<<" _conf_n_bins: "<<_conf_n_bins<<" _conf_max_slope: "
            <<_conf_max_slope<<" _conf_max_error: "<<_conf_max_error
           <<" _conf_long_threshold: "<<_conf_long_threshold<<" _conf_max_long_height: "
          <<_conf_max_long_height<<" _conf_max_start_height: "<<_conf_max_start_height
         <<" _conf_sensor_height: "<<_conf_sensor_height<<" _conf_k_margin: "<<_conf_k_margin;;
    return true;
}
bool Segment::fitSegmentLines() {
    lines_.clear();
    //Find first point.
    Bin::MinZPoint cur_point;
    auto line_start = bins_.begin();
    int cnt = 0;
    for (auto line_start = bins_.begin();line_start != bins_.end();++line_start) {
        if (line_start->hasPoint()) {
            cur_point = line_start->getMinZPoint();
            if ((cur_point.z + _conf_sensor_height) < _conf_max_start_height) {
                break;
            }
        }
        cnt++;
    }
    if (line_start == bins_.end()) {
        return false;
    }
    // Fill lines.
    bool is_long_line = false;
    double cur_ground_height = -_conf_sensor_height;
    std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
    LocalLine cur_line = std::make_pair(0,0);
    LocalLine new_line;
    for (auto line_iter = line_start + 1; line_iter != bins_.end(); ++line_iter) {
//        LOG(WARNING)<<" minzpoint d: "<<line_iter->getMinZPoint().d<<" minzpoint z: "
//                   <<line_iter->getMinZPoint().z;
        cnt++;
        if (line_iter->hasPoint()) {
            cur_point = line_iter->getMinZPoint();
            if (cur_point.d - current_line_points.back().d > _conf_long_threshold) {
                is_long_line = true;
            }
            if (current_line_points.size() >= 2) {
                // Get expected z value to possibly reject far away points.
                double expected_z = std::numeric_limits<double>::max();
                if (is_long_line && current_line_points.size() > 2) {
                    expected_z = cur_line.first * cur_point.d + cur_line.second;
                }
                current_line_points.push_back(cur_point);
                cur_line = fitLocalLine(current_line_points);
                double error = getMaxError(current_line_points, cur_line);
                if ((error > _conf_max_error) ||
                        (std::fabs(cur_line.first) > _conf_max_slope) ||
                        (is_long_line && ((std::fabs(expected_z - cur_point.z) > _conf_max_long_height)))) {
                    // Add line until previous point as ground.
                    current_line_points.pop_back();
                    // Don't let lines with 2 base points through.
                    if (current_line_points.size() >= 3) {
                        //common::time::Timer proc_timer;
                        new_line = fitLocalLine(current_line_points);
                        //LOG(ERROR)<<" fitline proc elapsed: "<<proc_timer.elapsed()<<" ms";
                        lines_.push_back(localLineToLine(new_line, current_line_points));
                        cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
                    }
                    // Start new line.
                    is_long_line = false;
                    current_line_points.erase(current_line_points.begin(), --current_line_points.end());
                    --line_iter;
                }else {
                    // Good line, continue.
                }
            }else {
                // Not enough points.
                if (((cur_point.d - current_line_points.back().d) < _conf_long_threshold) &&
                        (std::fabs(current_line_points.back().z - cur_ground_height) < _conf_max_start_height)) {
                    //Add point if valid.
                    current_line_points.push_back(cur_point);
                }else {
                    // Start new line.
                    current_line_points.clear();
                    current_line_points.push_back(cur_point);
                }
            }
        }
        //LOG(ERROR)<<" bin nums: "<<cnt<<" current line points num: "<<current_line_points.size();
    }
    // Add last line.
    if (current_line_points.size() > 2) {
        new_line = fitLocalLine(current_line_points);
        lines_.push_back(localLineToLine(new_line, current_line_points));
    }

//    LOG(WARNING)<<" lines_ size: "<<lines_.size();
//    for (auto it = lines_.begin();it != lines_.end();++it) {
//        LOG(WARNING)<<"first point d: "<<it->first.d<<" first point z: "<<it->first.z
//                <<" second point d: "<<it->second.d<<" second point z: "<<it->second.z;
//    }
    return true;
}
Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<Bin::MinZPoint>& line_points) {
    Line line;
    double first_d = line_points.front().d;
    double second_d = line_points.back().d;
    double first_z = local_line.first * first_d + local_line.second;
    double second_z = local_line.first * second_d + local_line.second;
    line.first.z = first_z;
    line.first.d = first_d;
    line.second.z = second_z;
    line.second.d = second_d;
    return line;
}
double Segment::verticalDistanceToLine(const double &d, const double &z) {
    double distance = -1;
    for (auto it = lines_.begin(); it != lines_.end(); ++it) {
        if (it->first.d - _conf_k_margin < d && it->second.d + _conf_k_margin > d) {
            double delta_z = it->second.z - it->first.z;
            double delta_d = it->second.d - it->first.d;
            double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;
            distance = std::fabs(z - expected_z);
        }
    }
    return distance;
}

double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
    double error_sum = 0;
    for (auto it = points.begin(); it != points.end(); ++it) {
        double residual = (line.first * it->d + line.second) - it->z;
        error_sum += residual * residual;
    }
    return error_sum/points.size();
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
    double max_error = 0;
    for (auto it = points.begin(); it != points.end(); ++it) {
        double residual = (line.first * it->d + line.second) - it->z;
        double error = residual * residual;
        if (error > max_error) max_error = error;
    }
    return max_error;
}
Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points) {
    Eigen::MatrixXd X(points.size(), 2);
    Eigen::VectorXd Y(points.size());
    size_t counter = 0;
    for (auto iter = points.begin(); iter != points.end(); ++iter) {
        X(counter, 0) = iter->d;
        X(counter, 1) = 1;
        Y(counter) = iter->z;
        ++counter;
    }
    Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
    LocalLine line_result;
    line_result.first = result(0);
    line_result.second = result(1);
    return line_result;
}

bool Segment::getLines(std::list<Line> *lines) {
    if (lines_.empty()) {
        return false;
    }
    else {
        *lines = lines_;
        return true;
    }
}
bool Segment::td_equal(const double a,const double b)
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
