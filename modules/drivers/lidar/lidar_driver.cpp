/**
* lidar_driver.cpp
* Author: zhubin
* Created on: 2020-06-17
* Copyright (c) iRotran. All Rights Reserved
*/
#include "lidar_driver.h"
#include "modules/framework/conf/parameter.h"
#include "modules/common/readyaml/readyaml.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/time/timer.h"
#include "modules/common/time/timestamp.h"
#include "modules/manager/mutex/mutexmanager.h"
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
namespace lidar_algorithm {
namespace drivers {
namespace lidar {
const std::string conf_path = "../modules/drivers/lidar/conf/lidar_conf.yaml";
LidarDriver::LidarDriver()
{

}
LidarDriver::~LidarDriver()
{

}
bool LidarDriver::load_conf(const std::string& conf_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(conf_path);
    std::string default_path = "./";
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"load_pcd_path",this->_conf_load_pcd_path,default_path);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"load_pcd_path conf failed!";
    }else{
        LOG(INFO)<<"load_pcd_path: "<<_conf_load_pcd_path;
    }
    readyaml.parse_yaml(parma_node,"data_cyclical_pattern",this->_conf_data_cyclical_pattern,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"data_cyclical_pattern conf failed!";
    }else{
        LOG(INFO)<<"data_cyclical_pattern: "<<_conf_data_cyclical_pattern;
    }
    readyaml.parse_yaml(parma_node,"use_passthrough_filter",this->_conf_use_passthrough_filter,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"use_passthrough_filter conf failed!";
    }else{
        LOG(INFO)<<"use_passthrough_filter: "<<_conf_use_passthrough_filter;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","x","left",this->_conf_lidar_roi_x_left,-5.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_x_left conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_x_left: "<<_conf_lidar_roi_x_left;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","x","right",this->_conf_lidar_roi_x_right,5.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_x_right conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_x_right: "<<_conf_lidar_roi_x_right;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","y","front",this->_conf_lidar_roi_y_front,0.2f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_y_front conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_y_front: "<<_conf_lidar_roi_y_front;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","y","after",this->_conf_lidar_roi_y_after,30.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_y_after conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_y_after: "<<_conf_lidar_roi_y_after;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","z","down",this->_conf_lidar_roi_z_down,-0.2f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_z_down conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_z_down: "<<_conf_lidar_roi_z_down;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","z","up",this->_conf_lidar_roi_z_up,2.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_lidar_roi_z_up conf failed!";
    }else{
        LOG(INFO)<<"_lidar_roi_z_up: "<<_conf_lidar_roi_z_up;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","intensity","min",this->_conf_lidar_roi_intensity_min,0.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_conf_lidar_roi_intensity_min conf failed!";
    }else{
        LOG(INFO)<<"_conf_lidar_roi_intensity_min: "<<_conf_lidar_roi_intensity_min;
    }
    readyaml.parse_yaml(parma_node,"lidar_roi","intensity","max",this->_conf_lidar_roi_intensity_max,100.0f);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"_conf_lidar_roi_intensity_max conf failed!";
    }else{
        LOG(INFO)<<"_conf_lidar_roi_intensity_max: "<<_conf_lidar_roi_intensity_max;
    }
    return true;
}
bool LidarDriver::load_pcd(const std::string& file_path)
{
    _pcl_pointcloud.clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path,_pcl_pointcloud) == -1) {
        LOG(FATAL)<<"load pcd file failed!";
    }
    long long time_stamp = common::time::TimeStamp::get_obj()->get_now_timestamp_system(common::time::_milliseconds);
    if (_conf_use_passthrough_filter) {
        pass_filter();
        _pcformatconv.pcl_proto(_pcl_pointcloud_filtered,_proto_pointcloud);
    }else{
        _pcformatconv.pcl_proto(_pcl_pointcloud,_proto_pointcloud);
    }
    _proto_pointcloud.set_timestamp(time_stamp);
    return true;
}
bool LidarDriver::pass_filter()
{
    pcl::IndicesPtr indices_x(new std::vector<int>);
    pcl::IndicesPtr indices_xy(new std::vector<int>);
    pcl::IndicesPtr indices_xyz(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZI> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    ptfilter.setInputCloud(_pcl_pointcloud.makeShared());
    //filter x
    ptfilter.setFilterFieldName ("x");//设置过滤时所需要点云类型的x字段
    // The indices_x array indexes all points of cloud_in that have x between _conf_lidar_roi_x_left and _conf_lidar_roi_x_right
    ptfilter.setFilterLimits (_conf_lidar_roi_x_left,_conf_lidar_roi_x_right);
    ptfilter.filter (*indices_x);
    //filter xy
    ptfilter.setIndices (indices_x);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (_conf_lidar_roi_y_front,_conf_lidar_roi_y_after);
    ptfilter.setNegative (false);
    ptfilter.filter (*indices_xy);
    //filter xyz
    ptfilter.setIndices (indices_xy);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (_conf_lidar_roi_z_down,_conf_lidar_roi_z_up);
    ptfilter.setNegative (false);
    ptfilter.filter (*indices_xyz);
    //filter xyzi
    ptfilter.setIndices (indices_xyz);
    ptfilter.setFilterFieldName ("intensity");
    ptfilter.setFilterLimits (_conf_lidar_roi_intensity_min,_conf_lidar_roi_intensity_max);
    ptfilter.setNegative (false);
    ptfilter.filter (_pcl_pointcloud_filtered);
    return true;
}
bool LidarDriver::pubilc_data(common::proto::PointCloud* const lidar_cloud)
{
    manager::mutex::MutexManager *mutexmanager_obj = manager::mutex::MutexManager::get_obj();
    {//写锁定，unique_lock
        boost::unique_lock<boost::shared_mutex> lidar_write(mutexmanager_obj->_lidar_data_shmutex);    //写锁定，unique_lock
        *lidar_cloud = _proto_pointcloud;
    }//写锁定,结束
    return true;
}
bool LidarDriver::init(common::proto::PointCloud* const lidar_cloud)
{
    //load conf
    load_conf(conf_path);
    //load pcd path
    _readtext.read_text_line(_conf_load_pcd_path,_pcdpath_vec);
    _pcd_nums = _pcdpath_vec.size();
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    _lidar_thread = std::thread(&LidarDriver::thread_proc,this,lidar_cloud);
    if (conf_obj->_sensor_thread_join){
        _lidar_thread.join();
    }else{
        _lidar_thread.detach();
    }
    return true;
}
bool LidarDriver::thread_proc(common::proto::PointCloud * const lidar_cloud)
{
    LOG(INFO)<<"lidar driver thread ready! ";
    std::cout <<"lidar driver thread ready!"<<std::endl;
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    long long cnt = 0;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    while(true){
        common::time::Timer proc_timer;
        if (cnt < _pcd_nums) {
            load_pcd(_pcdpath_vec[cnt]);
        }else{
            if (_conf_data_cyclical_pattern) {
                cnt = -1;
            }else{
                load_pcd(_pcdpath_vec[_pcd_nums-1]);
            }
        }
        pubilc_data(lidar_cloud);
        cnt++;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(conf_obj->_lidar_period));
        if (conf_obj->_use_data_log) {
            if (cnt%conf_obj->_log_save_freq == 0){
                LOG(INFO)<<"LidarDriver Modules: "
                        <<" cnt= "<<cnt
                       << " process elapsed= "<<proc_timer.elapsed() << " ms";
            }
        }
    }
    return true;
}
}//namespace lidar
}//namespace drivers
}//namespace lidar_algorithm
