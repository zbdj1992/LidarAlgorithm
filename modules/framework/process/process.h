/**
* process.h
* Author: zhubin
* Created on: 2018-09-01
* Copyright (c) iRotran. All Rights Reserved
*/
#pragma once
#include "modules/pclshow3d/pcl_show3d.h"
#include "modules/lidarlib/points_process.h"
namespace lidar_algorithm {
namespace framework {
namespace process {
class Process
{
public:
    Process();
    ~Process();
public:
    int init();
    int process();
public:
	static Process* get_obj() {
		static Process process_obj;
		return &process_obj;
	}
private:
    pcl_show::PclShow3D *_pclshow;
    points_process::PointsProcess *_points_process;
};
}//namespace process
}//namespace framework
}//namespace lidar_algorithm
