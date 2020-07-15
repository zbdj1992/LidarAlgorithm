/**
* parameter.h
* Author: zhubin
* Created on: 2018-09-01
* Copyright (c) iRotran. All Rights Reserved
*/
#include "parameter.h"
#include <memory.h>
#include <string.h>
#include <stdio.h>
#include "modules/common/log/glogutil.h"
using std::string;
using std::map;
using std::pair;
namespace lidar_algorithm {
namespace framework {
namespace conf {
Parameter::Parameter()
{
    _use_gtest = 0;
    _offline_data = 0;
	_hdmap_path = "";
	_use_perception = 0;
	_use_decision = 0;
	_use_control = 0;
    _use_diagnosis = 0;
    _use_localization = 0;
    _use_hmi = 0;
    _use_mapinfo = 0;
	_use_gps = 0;
    _use_canbus_send = 0;
	_use_veh = 0;
    _use_radar_side = 0;
    _use_radar_forward = 0;
	_use_lidar = 0;
	_use_ifcu = 0;
	_use_cam = 0;
    _use_sst = 0;
    _sensor_thread_join = 0;
    _voice_play_thread_join = 0;
    _data_show_thread_join = 0;
    _startup_diagnostics_thread_join = 0;
    _localization_thread_join = 0;
    _perception_thread_join = 0;
    _decision_thread_join = 0;
    _control_thread_join = 0;
    _diagnosis_thread_join = 0;
    _hmi_thread_join = 0;
    _mapinfo_thread_join = 0;
    _use_data_show = 0;
    _use_voice_play = 0;
    _use_data_diagnostics = 0;
    _use_data_log = 0;
    _log_save_freq = 1;
    _perception_period = 50;
    _decision_period = 50;
    _control_period = 50;
    _localization_period = 50;
    _mapinfo_period = 50;
    _hmi_period = 50;
    _diagnosis_period = 50;
    _veh_period = 20;
    _networkcan_period = 5;
    _use_percfusion = 0;
    _percfusion_thread_join = 0;
    _percfusion_period = 50;
    _use_planning = 0;
    _planning_period = 20;
    _planning_thread_join = 0;
    _tx2_can0 = "";
    _tx2_can1 = "";
    _use_tx2can_0 = 0;
    _use_tx2can_1 = 0;
    _use_v2x = 0;
    _v2x_thread_join = 0;
    _v2x_period = 1000;
    _use_networkcan = 0;
    _use_softwaretest = 0;
    _softwaretest_thread_join = 0;
    _softwaretest_period = 200;
    _self_check_time_limit = 1000;
    _use_sysdatagrep = 0;
    _sysdatagrep_thread_join = 0;
    _sysdatagrep_period = 100;
    _lidar_period = 100;
    _use_pclshow3d = 0;
    _pclshow3d_period = 20;
    _use_lidarlib = 0;
    _lidarlib_thread_join = 0;
    _lidarlib_period = 10;
}

Parameter::~Parameter()
{
}

int Parameter::init(const char *config_file)
{
    load_conf(config_file);
    parm_log();
    return RET_OK;
}
int Parameter::load_conf(const char *config_file)
{
    if (config_file == NULL || config_file[0] == '\0') {
        LOG(FATAL)<<"main.conf file not set!";
    }
    int ret = load(config_file);
    if (ret == -1) {
        LOG(FATAL)<<"load config file %s failed!";
    }
    LOG(INFO)<<"main.conf file set success!";
    map<string, string>::iterator iter;
    iter = _map_vec.find("USE_GTEST");
    if (iter != _map_vec.end()) {
        _use_gtest = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("OFFLINE_DATA");
    if (iter != _map_vec.end()) {
        _offline_data = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("HDMAP_PATH");
    if (iter != _map_vec.end()) {
		_hdmap_path = iter->second.c_str();
    }
    iter = _map_vec.find("USE_PERCEPTION");
    if (iter != _map_vec.end()) {
        _use_perception = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_DECISION");
    if (iter != _map_vec.end()) {
        _use_decision = atoi(iter->second.c_str());
    }
	iter = _map_vec.find("USE_CONTROL");   //zb
    if (iter != _map_vec.end()) {
        _use_control = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_DIAGNOSIS");   //zb
    if (iter != _map_vec.end()) {
        _use_diagnosis = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_LOCALIZATION");
    if (iter != _map_vec.end()) {
        _use_localization = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_PCLSHOW3D");
    if (iter != _map_vec.end()) {
        _use_pclshow3d = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_LIDARLIB");
    if (iter != _map_vec.end()) {
        _use_lidarlib = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_HMI");
    if (iter != _map_vec.end()) {
        _use_hmi = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_MAPINFO");
    if (iter != _map_vec.end()) {
        _use_mapinfo = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_PERCFUSION");
    if (iter != _map_vec.end()) {
        _use_percfusion = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_PLANNING");
    if (iter != _map_vec.end()) {
        _use_planning = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_V2X");
    if (iter != _map_vec.end()) {
        _use_v2x = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_SOFTWARETEST");
    if (iter != _map_vec.end()) {
        _use_softwaretest = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_SYSDATAGREP");
    if (iter != _map_vec.end()) {
        _use_sysdatagrep = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_GPS");
    if (iter != _map_vec.end()) {
        _use_gps = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_VEH");
    if (iter != _map_vec.end()) {
        _use_veh = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_NETWORK_CAN");
    if (iter != _map_vec.end()) {
        _use_networkcan = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_USS");
    if (iter != _map_vec.end()) {
        _use_uss = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_SST");
    if (iter != _map_vec.end()) {
        _use_sst = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_LIDAR");
    if (iter != _map_vec.end()) {
        _use_lidar = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_CAM");
    if (iter != _map_vec.end()) {
        _use_cam = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_IFCU");
    if (iter != _map_vec.end()) {
        _use_ifcu = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_TX2CAN_0");
    if (iter != _map_vec.end()) {
        _use_tx2can_0 = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_TX2CAN_1");
    if (iter != _map_vec.end()) {
        _use_tx2can_1 = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_RADAR_SIDE");
    if (iter != _map_vec.end()) {
        _use_radar_side = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_RADAR_FORWARD");
    if (iter != _map_vec.end()) {
        _use_radar_forward = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("SENSOR_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _sensor_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PCLSHOW3D_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _pclshow3d_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LIDARLIB_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _lidarlib_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("VOICEPLAY_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _voice_play_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("DATASHOW_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _data_show_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("START_UP_DIAGNOSTICS_JOIN");
    if (iter != _map_vec.end()) {
        _startup_diagnostics_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LOCALIZATION_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _localization_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PERCEPTION_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _perception_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("DECISION_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _decision_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("CONTROL_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _control_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("DIAGNOSTICS_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _diagnosis_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("HMI_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _hmi_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("MAPINFO_JOIN");
    if (iter != _map_vec.end()) {
        _mapinfo_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PERCFUSION_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _percfusion_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PLANNING_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _planning_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("V2X_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _v2x_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("SOFTWARETEST_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _softwaretest_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("SYSDATAGREP_THREAD_JOIN");
    if (iter != _map_vec.end()) {
        _sysdatagrep_thread_join = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_DATA_SHOW");
    if (iter != _map_vec.end()) {
        _use_data_show = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_DATA_DIAGNOSTICS");
    if (iter != _map_vec.end()) {
        _use_data_diagnostics = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_VOICE_PLAY");
    if (iter != _map_vec.end()) {
        _use_voice_play = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_CANBUS_SEND");
    if (iter != _map_vec.end()) {
        _use_canbus_send = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("USE_DATA_LOG");
    if (iter != _map_vec.end()) {
        _use_data_log = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LOG_SAVE_FREQ");
    if (iter != _map_vec.end()) {
        _log_save_freq = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PCLSHOW3D_PERIOD");
    if (iter != _map_vec.end()) {
        _pclshow3d_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LIDARLIB_PERIOD");
    if (iter != _map_vec.end()) {
        _lidarlib_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PERCEPTION_PERIOD");
    if (iter != _map_vec.end()) {
        _perception_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("DECISION_PERIOD");
    if (iter != _map_vec.end()) {
        _decision_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("CONTROL_PERIOD");
    if (iter != _map_vec.end()) {
        _control_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LOCALIZATION_PERIOD");
    if (iter != _map_vec.end()) {
        _localization_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("MAPINFO_PERIOD");
    if (iter != _map_vec.end()) {
        _mapinfo_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("HMI_PERIOD");
    if (iter != _map_vec.end()) {
        _hmi_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("DIAGNOSTICS_PERIOD");
    if (iter != _map_vec.end()) {
        _diagnosis_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PERCFUSION_PERIOD");
    if (iter != _map_vec.end()) {
        _percfusion_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("PLANNING_PERIOD");
    if (iter != _map_vec.end()) {
        _planning_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("V2X_PERIOD");
    if (iter != _map_vec.end()) {
        _v2x_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("SOFTWARETEST_PERIOD");
    if (iter != _map_vec.end()) {
        _softwaretest_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("SYSDATAGREP_PERIOD");
    if (iter != _map_vec.end()) {
        _sysdatagrep_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("VEH_PERIOD");
    if (iter != _map_vec.end()) {
        _veh_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("NETWORKCAN_PERIOD");
    if (iter != _map_vec.end()) {
        _networkcan_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("LIDAR_PERIOD");
    if (iter != _map_vec.end()) {
        _lidar_period = atoi(iter->second.c_str());
    }
    iter = _map_vec.find("TX2_CAN0");
    if (iter != _map_vec.end()) {
        _tx2_can0 = iter->second.c_str();
    }
    iter = _map_vec.find("TX2_CAN1");
    if (iter != _map_vec.end()) {
        _tx2_can1 = iter->second.c_str();
    }
    iter = _map_vec.find("SELF_CHECK_TIME_LIMIT");
    if (iter != _map_vec.end()) {
        _self_check_time_limit = atoi(iter->second.c_str());
    }
    return RET_OK;
}

int Parameter::print()
{
    printf("---------------- main config ----------------\n");
    printf("OFFLINE_DATA        : %d\n", _offline_data);
    printf("HDMAP_PATH          : %s\n", _hdmap_path.c_str());
	printf("USE_PERCEPTION      : %d\n", _use_perception);
	printf("USE_DECISION        : %d\n", _use_decision);
    printf("\n");
    return RET_OK;
}
int Parameter::parm_log()
{
    LOG(INFO)<<"USE_GTEST: "<<_use_gtest;
    LOG(INFO)<<"OFFLINE_DATA: "<<_offline_data;
    LOG(INFO)<<"USE_DATA_LOG: "<<_use_data_log;
    LOG(INFO)<<"LOG_SAVE_FREQ: "<<_log_save_freq;
    LOG(INFO)<<"HDMAP_PATH: "<<_hdmap_path;
    LOG(INFO)<<"USE_LOCALIZATION: "<<_use_localization;
    LOG(INFO)<<"USE_MAPINFO: "<<_use_mapinfo;
    LOG(INFO)<<"USE_PERCEPTION: "<<_use_perception;
    LOG(INFO)<<"USE_DECISION: "<<_use_decision;
    LOG(INFO)<<"USE_HMI: "<<_use_hmi;
    LOG(INFO)<<"USE_CONTROL: "<<_use_control;
    LOG(INFO)<<"USE_DIAGNOSIS: "<<_use_diagnosis;
    LOG(INFO)<<"USE_PERCFUSION: "<<_use_percfusion;
    LOG(INFO)<<"USE_PLANNING: "<<_use_planning;
    LOG(INFO)<<"USE_V2X: "<<_use_v2x;
    LOG(INFO)<<"USE_SYSDATAGREP: "<<_use_sysdatagrep;
    LOG(INFO)<<"USE_SOFTWARETEST: "<<_use_softwaretest;
    LOG(INFO)<<"USE_GPS: "<<_use_gps;
    LOG(INFO)<<"USE_RADAR_SIDE: "<<_use_radar_side;
    LOG(INFO)<<"USE_RADAR_FORWARD: "<<_use_radar_forward;
    LOG(INFO)<<"USE_VEH: "<<_use_veh;
    LOG(INFO)<<"USE_NETWORK_CAN: "<<_use_networkcan;
    LOG(INFO)<<"USE_USS: "<<_use_uss;
    LOG(INFO)<<"USE_SST: "<<_use_sst;
    LOG(INFO)<<"USE_LIDAR: "<<_use_lidar;
    LOG(INFO)<<"USE_CAM: "<<_use_cam;
    LOG(INFO)<<"USE_IFCU: "<<_use_ifcu;
    LOG(INFO)<<"USE_TX2CAN_0: "<<_use_tx2can_0;
    LOG(INFO)<<"USE_TX2CAN_1: "<<_use_tx2can_1;
    LOG(INFO)<<"USE_CANBUS_SEND: "<<_use_canbus_send;
    LOG(INFO)<<"USE_DATA_SHOW: "<<_use_data_show;
    LOG(INFO)<<"USE_VOICE_PLAY: "<<_use_voice_play;
    LOG(INFO)<<"USE_DATA_DIAGNOSTICS: "<<_use_data_diagnostics;
    LOG(INFO)<<"SENSOR_THREAD_JOIN: "<<_sensor_thread_join;
    LOG(INFO)<<"VOICEPLAY_THREAD_JOIN: "<<_voice_play_thread_join;
    LOG(INFO)<<"DATASHOW_THREAD_JOIN: "<<_data_show_thread_join;
    LOG(INFO)<<"START_UP_DIAGNOSTICS_JOIN: "<<_startup_diagnostics_thread_join;
    LOG(INFO)<<"LOCALIZATION_THREAD_JOIN: "<<_localization_thread_join;
    LOG(INFO)<<"MAPINFO_JOIN: "<<_mapinfo_thread_join;
    LOG(INFO)<<"PERCEPTION_THREAD_JOIN: "<<_perception_thread_join;
    LOG(INFO)<<"PERCFUSION_THREAD_JOIN: "<<_percfusion_thread_join;
    LOG(INFO)<<"DECISION_THREAD_JOIN: "<<_decision_thread_join;
    LOG(INFO)<<"PLANNING_THREAD_JOIN: "<<_planning_thread_join;
    LOG(INFO)<<"HMI_THREAD_JOIN: "<<_hmi_thread_join;
    LOG(INFO)<<"CONTROL_THREAD_JOIN: "<<_control_thread_join;
    LOG(INFO)<<"DIAGNOSTICS_THREAD_JOIN: "<<_diagnosis_thread_join;
    LOG(INFO)<<"V2X_THREAD_JOIN: "<<_v2x_thread_join;
    LOG(INFO)<<"SYSDATAGREP_THREAD_JOIN: "<<_sysdatagrep_thread_join;
    LOG(INFO)<<"SOFTWARETEST_THREAD_JOIN: "<<_softwaretest_thread_join;
    LOG(INFO)<<"PERCEPTION_PERIOD: "<<_perception_period;
    LOG(INFO)<<"DECISION_PERIOD: "<<_decision_period;
    LOG(INFO)<<"CONTROL_PERIOD: "<<_control_period;
    LOG(INFO)<<"LOCALIZATION_PERIOD: "<<_localization_period;
    LOG(INFO)<<"MAPINFO_PERIOD: "<<_mapinfo_period;
    LOG(INFO)<<"HMI_PERIOD: "<<_hmi_period;
    LOG(INFO)<<"DIAGNOSTICS_PERIOD: "<<_diagnosis_period;
    LOG(INFO)<<"PERCFUSION_PERIOD: "<<_percfusion_period;
    LOG(INFO)<<"PLANNING_PERIOD: "<<_planning_period;
    LOG(INFO)<<"V2X_PERIOD: "<<_v2x_period;
    LOG(INFO)<<"SYSDATAGREP_PERIOD: "<<_sysdatagrep_period;
    LOG(INFO)<<"SOFTWARETEST_PERIOD: "<<_softwaretest_period;
    LOG(INFO)<<"VEH_PERIOD: "<<_veh_period;
    LOG(INFO)<<"NETWORKCAN_PERIOD: "<<_networkcan_period;
    LOG(INFO)<<"TX2_CAN0: "<<_tx2_can0;
    LOG(INFO)<<"TX2_CAN1: "<<_tx2_can1;
    LOG(INFO)<<"SELF_CHECK_TIME_LIMIT: "<<_self_check_time_limit;
    return RET_OK;
}
int Parameter::load(const char *config_file)
{
    FILE *fp = fopen(config_file, "rt");
    if (fp == NULL) {
        LOG(ERROR)<<"Cannot open file"<<config_file;
        //printf("Cannot open file %s to read!\n", config_file);
        return RET_ERROR;
    }
    char content[1024];
    char res[1024];
    while (!feof(fp)) {
        memset(content, '\0', 1024);
        memset(res,     '\0', 1024);
        fgets(content, 1024, fp);
        char *temp = strtok(content, "\r\n");
        if (temp == NULL || temp[0] == '#' || temp[0] == '[') {
            continue;
        }
        int len = strlen(temp);
        int idx = 0;
        for (int ii = 0; ii < len; ii++) {
            if (temp[ii] == ' ') {
                continue;
            }
            res[idx] = temp[ii];
            idx++;
        }
        string str(res);
        int pos = str.find(":");
        if (pos == -1) {
            LOG(ERROR)<<"Wrong config format:"<<temp;
            //printf("Wrong config format: %s\n", temp);
            return RET_ERROR;
        }
        _map_vec.insert(pair<string, string>(str.substr(0, pos), str.substr(pos+1)));
    }
    fclose(fp);
    return RET_OK;
}
}//namespace conf
}//namespace framework
}//namespace lidar_algorithm
