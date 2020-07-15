/**
* pcl_show3d.cpp
* Author: zhubin
* Created on: 2020-06-18
* Copyright (c) iRotran. All Rights Reserved
*/
#include "pcl_show3d.h"
#include "modules/framework/conf/parameter.h"
#include "modules/common/time/timer.h"
#include "modules/common/time/timestamp.h"
#include "modules/common/log/glogutil.h"
#include "modules/common/readyaml/readyaml.h"
namespace lidar_algorithm {
namespace pcl_show {
const std::string conf_path = "../modules/pclshow3d/conf/pclshow_conf.yaml";
const std::string str_id_1 = "Overhead Cloud";
const std::string str_id_2 = "Ground Cloud";
PclShow3D::PclShow3D()
{

}
PclShow3D::~PclShow3D()
{

}
bool PclShow3D::load_conf(const std::string& file_path)
{
    using namespace common::readyaml;
    ReadYaml readyaml;
    YAML::Node parma_node = readyaml.load_conf(file_path);
    int ret = 0;
    ret = readyaml.parse_yaml(parma_node,"cloud_show",this->_conf_cloud_show,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"cloud_show conf failed!";
    }else{
        LOG(INFO)<<"cloud_show = "<<this->_conf_cloud_show;
    }
    readyaml.parse_yaml(parma_node,"aabb_show",this->_conf_aabb_show,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"aabb_show conf failed!";
    }else{
        LOG(INFO)<<"aabb_show = "<<this->_conf_aabb_show;
    }
    readyaml.parse_yaml(parma_node,"obb_show",this->_conf_obb_show,0);
    if (ret == RET_ERROR){
        LOG(FATAL)<<"obb_show conf failed!";
    }else{
        LOG(INFO)<<"obb_show = "<<this->_conf_obb_show;
    }
    return true;
}
pcl::visualization::PCLVisualizer::Ptr PclShow3D::init_pclvis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                                                              pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_cloud)
{
    //创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
    viewer->setBackgroundColor (0, 0, 0);
    /*将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，
     * 每调用一次就会创建一个新的ID号，如果想更新一个已经显示的点云，先调用removePointCloud（），并提供需要更新的点云ID 号，也可使用updatePointCloud*/
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(cloud, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ground_cloud_color(ground_cloud, 255, 255, 0); // yellow
    viewer->addPointCloud<pcl::PointXYZI> (cloud,cloud_color,str_id_1);
    viewer->addPointCloud<pcl::PointXYZI> (ground_cloud,ground_cloud_color,str_id_2);
    //用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法,1设置显示点云大小
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,str_id_1);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,str_id_2);
    viewer->addCoordinateSystem (1.0);
    //通过设置照相机参数使得从默认的角度和方向观察点云
    viewer->initCameraParameters ();
    return (viewer);
}
bool PclShow3D::update_pclvis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                   pcl::PointCloud<pcl::PointXYZI>::ConstPtr ground_cloud)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(cloud, 0, 255, 0); // green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ground_cloud_color(ground_cloud, 255, 255, 0); // yellow
    _viewer_ptr->updatePointCloud<pcl::PointXYZI>(cloud,cloud_color,str_id_1);
    _viewer_ptr->updatePointCloud<pcl::PointXYZI>(ground_cloud,ground_cloud_color,str_id_2);
    return true;
}
bool PclShow3D::add_cube(const framework::data::proto::LidarLibData& lidarlib_data)
{
    _viewer_ptr->removeAllShapes();
    int aabb_cnt = 0;
    int obb_cnt = 0;
    int aabbs_num = lidarlib_data.aabb3ds().aabb3ds_size();
    int obbs_num = lidarlib_data.obb3ds().obb3ds_size();
    //AABB
    if (_conf_aabb_show) {
        for (int k = 0;k<aabbs_num;k++) {
            _viewer_ptr->addCube(lidarlib_data.aabb3ds().aabb3ds(k).min_point().x(),
                                 lidarlib_data.aabb3ds().aabb3ds(k).max_point().x(),
                                 lidarlib_data.aabb3ds().aabb3ds(k).min_point().y(),
                                 lidarlib_data.aabb3ds().aabb3ds(k).max_point().y(),
                                 lidarlib_data.aabb3ds().aabb3ds(k).min_point().z(),
                                 lidarlib_data.aabb3ds().aabb3ds(k).max_point().z(),
                                 1.0, 0.0, 0.0, "AABB"+std::to_string(aabb_cnt));
            aabb_cnt++;
        }
    }
    //OBB
    if (_conf_obb_show) {
        for (int k = 0;k<obbs_num;k++) {
            Eigen::Quaternionf quat(lidarlib_data.obb3ds().obb3ds(k).quat().w(),
                                    lidarlib_data.obb3ds().obb3ds(k).quat().x(),
                                    lidarlib_data.obb3ds().obb3ds(k).quat().y(),
                                    lidarlib_data.obb3ds().obb3ds(k).quat().z());
            Eigen::Vector3f position (lidarlib_data.obb3ds().obb3ds(k).position().x(),
                                      lidarlib_data.obb3ds().obb3ds(k).position().y(),
                                      lidarlib_data.obb3ds().obb3ds(k).position().z());
            double width = lidarlib_data.obb3ds().obb3ds(k).max_point().x() - lidarlib_data.obb3ds().obb3ds(k).min_point().x();
            double height = lidarlib_data.obb3ds().obb3ds(k).max_point().y() - lidarlib_data.obb3ds().obb3ds(k).min_point().y();
            double depth = lidarlib_data.obb3ds().obb3ds(k).max_point().z() - lidarlib_data.obb3ds().obb3ds(k).min_point().z();
            _viewer_ptr->addCube(position,quat,width,height,depth,"OBB"+std::to_string(obb_cnt));
            _viewer_ptr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                     1.0,0.0,1.0,"OBB"+std::to_string(obb_cnt));
            obb_cnt++;
        }
    }
    return true;
}
bool PclShow3D::spilt_cloud(const framework::data::proto::LidarLibData& lidarlib_data,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    ground_cloud->clear();
    cloud->clear();
    pcl::PointXYZI onepoint;
    int points_num = lidarlib_data.point_cloud().points_size();
    int cloud_cnt = 0;
    int ground_cloud_cnt = 0;
    for (int k = 0;k<points_num;++k) {
        onepoint.x = lidarlib_data.point_cloud().points(k).x();
        onepoint.y = lidarlib_data.point_cloud().points(k).y();
        onepoint.z = lidarlib_data.point_cloud().points(k).z();
        onepoint.intensity = lidarlib_data.point_cloud().points(k).intensity();
        if (lidarlib_data.point_cloud().points(k).label() == 1) {
            ground_cloud->points.push_back(onepoint);
            ground_cloud_cnt++;
        }else{
            cloud->points.push_back(onepoint);
            cloud_cnt++;
        }
    }
    ground_cloud->width = ground_cloud_cnt;
    ground_cloud->height = 1;
    cloud->width = cloud_cnt;
    cloud->height = 1;
    return true;
}
bool PclShow3D::init()
{
    //load conf
    load_conf(conf_path);
    //init PCLVisualizer
    return true;
}
bool PclShow3D::process()
{
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    _pclshow_thread = std::thread(&PclShow3D::thread_proc,this);
    if (conf_obj->_pclshow3d_thread_join){
        _pclshow_thread.join();
    }else{
        _pclshow_thread.detach();
    }
    return true;
}
bool PclShow3D::thread_proc(void)
{
    LOG(INFO)<<"pcl_show process thread ready! ";
    std::cout <<"pcl_show process thread ready!"<<std::endl;
    framework::conf::Parameter *conf_obj = framework::conf::Parameter::get_obj();
    framework::data::Data *data_obj = framework::data::Data::get_obj();
    //common::proto::PointCloud proto_cloud_data;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    framework::data::proto::LidarLibData lidarlib_data;
    //init PCLVisualizer
    _viewer_ptr = init_pclvis(pcl_cloud,pcl_ground_cloud);
    long long cnt = 0;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    while(!_viewer_ptr->wasStopped()){
        common::time::Timer proc_timer;
        //proto_cloud_data = data_obj->get_one_pointcloud();
        lidarlib_data = data_obj->get_one_lidarlib_data();
        spilt_cloud(lidarlib_data,pcl_ground_cloud,pcl_cloud);
        //_pcformatconv.proto_pcl(proto_cloud_data,*pcl_cloud);
        add_cube(lidarlib_data);
        update_pclvis(pcl_cloud,pcl_ground_cloud);
        _viewer_ptr->spinOnce(1);
        cnt++;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(conf_obj->_pclshow3d_period));
        if (conf_obj->_use_data_log) {
            if (cnt%conf_obj->_log_save_freq == 0){
                LOG(INFO)<<"PclShow3D Modules: "
                        <<" cnt= "<<cnt
                       << " process elapsed= "<<proc_timer.elapsed() << " ms";
            }
        }
    }
    return true;
}
}//namespace pcl_show
}//namespace lidar_algorithm
