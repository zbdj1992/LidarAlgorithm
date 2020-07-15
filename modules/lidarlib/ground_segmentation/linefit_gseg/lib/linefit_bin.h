/**
* linefit_bin.h
* Author: zhubin
* Created on: 2020-07-03
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_
//#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace lidar_algorithm {
namespace points_process {
namespace linefit_gseg {
class Bin {
public:
    /*设定了一个最小Z点的结构体，其中是一个二维结构，在x-y平面的线段长度，以及z轴上的数值*/
    struct MinZPoint {
        MinZPoint() :
            z(std::numeric_limits<double>::max()),
            d(0) {}
        MinZPoint(const double& d, const double& z) : z(z), d(d) {}
        bool operator==(const MinZPoint& comp) {return z == comp.z && d == comp.d;}
        double z;
        double d;
    };
private:
    bool has_point_;
    MinZPoint minz_point_;
public:
    Bin();
    /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
    void addPoint(const pcl::PointXYZ& point);
    void addPoint(const double& d, const double& z);
    inline MinZPoint getMinZPoint(){return minz_point_;}
    inline bool hasPoint() {return has_point_;}
};
}//namespace linefit_gseg
}//namespace points_process
}//namespace lidar_algorithm
#endif /* GROUND_SEGMENTATION_BIN_H_ */
