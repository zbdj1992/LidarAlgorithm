# LidarAlgorithm  
3D lidar points cloud process  
1、Getting Started  
    离线开发3D 点云相关算法的框架，驱动层读取离线点云数据，算法层开发相应算法（地面点分割、聚类、BoundingBox、跟踪等），显示层显示点云数据及障碍物目标3D框。  
2、Prerequisites  
    * Ubuntu 16.04  
    * Qt Creator (编辑及编译 未依赖QT任何库)  
    * glog、gtest、protobuf、eigen、lcm、boost、yaml、pcl1.7、vtk、opencv  
3、Framework  
    参见docs/img/framework.jpg  
4、User Guide  
    * 离线数据准备  
        a、pcd数据  
        链接：https://pan.baidu.com/s/1wQ56ubUmr-Gn1ZXLF_PDqQ 
        提取码：lw0s  
        b、文件夹中pcd文件，路径+文件名—>列表文件文件（ls /path/*.pcd>data_pcd_list）  
        c、更改配置 modules/drivers/lidar/conf/lidar_conf.yaml  
        load_pcd_path : ../modules/drivers/lidar/data/pcd_data_list0619  
    * 3d show
        a、修改配置文件 modules/pclshow3d/conf/pclshow_conf.yaml  
        cloud_show : 1 # 0不显示点云数据 1显示点云数据  
        aabb_show : 0  # 0不显示AABB 1显示AABB  
        obb_show : 1   #0不显示OBB 1显示OBB  
        b、地面点云：黄色、障碍物点云：绿色  
           AABB：红色  
           OBB: 粉红色  
        显示效果docs/img  
        
