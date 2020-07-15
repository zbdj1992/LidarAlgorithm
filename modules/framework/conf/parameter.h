/**
* parameter.h
* Author: zhubin
* Created on: 2018-09-01
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef INC_PARAMETER_H
#define INC_PARAMETER_H

#include "modules/common/common.h"
#include <string>
#include <map>
namespace lidar_algorithm {
namespace framework {
namespace conf {
class Parameter {
public:
    Parameter();
    ~Parameter();
public:
    int init(const char *conf_file);
    static Parameter *get_obj() {
        static Parameter param_obj;
        return &param_obj;
    }

public:
    int _use_gtest;
    int _offline_data;
    int _use_pclshow3d;
    int _use_lidarlib;
    std::string _hdmap_path;
    /*perceprion decision localization control hmi mapinfo select*/
	int _use_perception;
	int _use_decision;
    int _use_control;
    int _use_localization;
    int _use_hmi;
    int _use_mapinfo;
    int _use_diagnosis;
    int _use_percfusion;
    int _use_planning;
    int _use_v2x;
    int _use_softwaretest;
    int _use_sysdatagrep;
    /*sensor select*/
    int _use_networkcan;
    int _use_veh;
    int _use_uss;
    int _use_sst;
    int _use_lidar;
    int _use_cam;
    int _use_ifcu;
    int _use_gps;
    int _use_tx2can_0;
    int _use_tx2can_1;
    int _use_canbus_send;
    int _use_radar_side;
    int _use_radar_forward;
    /*thread join or detach*/
    int _sensor_thread_join;
    int _pclshow3d_thread_join;
    int _lidarlib_thread_join;
    int _voice_play_thread_join;
    int _data_show_thread_join;
    int _startup_diagnostics_thread_join;
    int _localization_thread_join;
    int _perception_thread_join;
    int _decision_thread_join;
    int _control_thread_join;
    int _diagnosis_thread_join;
    int _hmi_thread_join;
    int _mapinfo_thread_join;
    int _percfusion_thread_join;
    int _planning_thread_join;
    int _v2x_thread_join;
    int _softwaretest_thread_join;
    int _sysdatagrep_thread_join;
    /*data show*/
    int _use_data_show;
    /*voice play*/
    int _use_voice_play;
    /*data log*/
    int _use_data_log;
    int _log_save_freq;
    /*sensor data diagnostics*/
    int _use_data_diagnostics;
    /*modules period*/
    int _pclshow3d_period;
    int _lidarlib_period;
    int _perception_period;
    int _decision_period;
    int _control_period;
    int _localization_period;
    int _mapinfo_period;
    int _hmi_period;
    int _diagnosis_period;
    int _percfusion_period;
    int _planning_period;
    int _v2x_period;
    int _softwaretest_period;
    int _sysdatagrep_period;
    /*sensor data period*/
    int _veh_period;
    int _networkcan_period;
    int _lidar_period;
    /*tx2 can name*/
    std::string _tx2_can0;
    std::string _tx2_can1;
    /*self-check time upper limit*/
    int _self_check_time_limit;
private:
    int load_conf(const char *config_file);
    int load(const char *config_file);
    int print();
    int parm_log();
private:
    std::map<std::string, std::string> _map_vec;

};
}//namespace conf
}//namespace framework
}//namespace lidar_algorithm

#endif

