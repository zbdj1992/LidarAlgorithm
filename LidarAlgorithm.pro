QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
    modules/common/log/glogutil.cpp \
    modules/common/proto/geometry.pb.cc \
    modules/common/time/timestamp.cpp \
    modules/common/time/timestamp_gtest.cpp \
    modules/framework/conf/parameter.cpp \
    modules/framework/data/data.cpp \
    modules/manager/mutex/mutexmanager.cpp \
    modules/framework/process/process.cpp \
    modules/drivers/lidar/lidar_driver.cpp \
    modules/common/util/pc_format_conv/pcformat_conv.cpp \
    modules/common/util/readtext/read_text.cpp \
    modules/common/util/readtext/read_text_test.cpp \
    modules/pclshow3d/pcl_show3d.cpp \
    modules/lidarlib/points_process.cpp \
    modules/framework/data/proto/lidarlib_modu_result.pb.cc \
    modules/lidarlib/segmentation/eucluster/eu_cluster.cpp \
    modules/lidarlib/filter/points_filter.cpp \
    modules/lidarlib/boxbuilder/box_builder.cpp \
    modules/lidarlib/segmentation/cluster_postprocessing/cluster_postprocessing.cpp \
    modules/lidarlib/segmentation/regcluster/reg_cluster.cpp \
    modules/lidarlib/segmentation/groundseg/ground_filter.cpp \
    modules/lidarlib/ground_segmentation/linefit_gseg/linefit_gseg.cpp \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_segment.cpp \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_gsegmentation.cpp \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_bin.cpp \
    modules/lidarlib/segmentation/ceucluster/ceu_cluster.cpp

HEADERS += \
    modules/common/log/glogutil.h \
    modules/common/proto/geometry.pb.h \
    modules/common/readyaml/readyaml.h \
    modules/common/threadpool/detail/future.hpp \
    modules/common/threadpool/detail/locking_ptr.hpp \
    modules/common/threadpool/detail/pool_core.hpp \
    modules/common/threadpool/detail/scope_guard.hpp \
    modules/common/threadpool/detail/worker_thread.hpp \
    modules/common/threadpool/future.hpp \
    modules/common/threadpool/pool.hpp \
    modules/common/threadpool/pool_adaptors.hpp \
    modules/common/threadpool/scheduling_policies.hpp \
    modules/common/threadpool/shutdown_policies.hpp \
    modules/common/threadpool/size_policies.hpp \
    modules/common/threadpool/task_adaptors.hpp \
    modules/common/threadpool/threadpool.hpp \
    modules/common/time/timer.h \
    modules/common/time/timestamp.h \
    modules/common/common.h \
    modules/framework/conf/parameter.h \
    modules/framework/data/data.h \
    modules/manager/mutex/mutexmanager.h \
    modules/framework/process/process.h \
    modules/drivers/lidar/lidar_driver.h \
    modules/common/util/pc_format_conv/pcformat_conv.h \
    modules/common/util/readtext/read_text.h \
    modules/pclshow3d/pcl_show3d.h \
    modules/lidarlib/points_process.h \
    modules/framework/data/proto/lidarlib_modu_result.pb.h \
    modules/lidarlib/interface/base_cluster.h \
    modules/lidarlib/segmentation/eucluster/eu_cluster.h \
    modules/lidarlib/interface/base_filter.h \
    modules/lidarlib/filter/points_filter.h \
    modules/lidarlib/interface/base_box_builder.h \
    modules/lidarlib/boxbuilder/box_builder.h \
    modules/lidarlib/interface/base_post_processing.h \
    modules/lidarlib/segmentation/cluster_postprocessing/cluster_postprocessing.h \
    modules/lidarlib/segmentation/regcluster/reg_cluster.h \
    modules/lidarlib/interface/base_ground_seg.h \
    modules/lidarlib/segmentation/groundseg/ground_filter.h \
    modules/lidarlib/interface/base_linefilt_gseg.h \
    modules/lidarlib/ground_segmentation/linefit_gseg/linefit_gseg.h \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_segment.h \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_gsegmentation.h \
    modules/lidarlib/ground_segmentation/linefit_gseg/lib/linefit_bin.h \
    modules/lidarlib/segmentation/ceucluster/ceu_cluster.h

DISTFILES += \
    modules/common/proto/geometry.proto \
    modules/framework/conf/main.conf \
    modules/drivers/lidar/conf/lidar_conf.yaml \
    modules/framework/data/proto/lidarlib_modu_result.proto \
    modules/lidarlib/conf/points_filter_conf.yaml \
    modules/lidarlib/conf/eu_cluster_conf.yaml \
    modules/pclshow3d/conf/pclshow_conf.yaml \
    modules/lidarlib/conf/cluster_postprocessing_conf.yaml \
    modules/lidarlib/conf/lidarlib_conf.yaml \
    modules/lidarlib/conf/reg_cluster_conf.yaml \
    modules/lidarlib/conf/linefit_gseg_conf.yaml
#x86_64-linux-gnu
#GLOG
INCLUDEPATH += /usr/local/include/glog
LIBS        += /usr/local/lib/libglog.so
# GTEST
INCLUDEPATH += /usr/include/gtest
LIBS        += /usr/lib/libgtest.a
#Eigen
INCLUDEPATH += /usr/include/eigen3
#LCM
INCLUDEPATH += /usr/local/include/lcm
LIBS        += /usr/local/lib/liblcm.so
#protobuf3.7.1
INCLUDEPATH += /usr/local/protobuf-3.7.1/include
LIBS += /usr/local/protobuf-3.7.1/lib/libprotobuf.so \
        /usr/local/protobuf-3.7.1/lib/libprotobuf-lite.so \
        /usr/local/protobuf-3.7.1/lib/libprotoc.so
#boost
INCLUDEPATH += /usr/include/boost
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_thread.a
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_system.a
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_chrono.a
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_date_time.a
#YAML
INCLUDEPATH += /usr/include/yaml-cpp
LIBS        += /usr/lib/x86_64-linux-gnu/libyaml*.so
#Vtk
INCLUDEPATH +=/usr/include/vtk-6.2
LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so
#PCL
INCLUDEPATH += /usr/include/pcl-1.7
LIBS        += /usr/lib/x86_64-linux-gnu/libpcl_*.so
#opencv
INCLUDEPATH += /usr/local/include/opencv \
               /usr/local/include/opencv2
LIBS        += /usr/local/lib/libopencv_core.so \
               /usr/local/lib/libopencv_highgui.so \
               /usr/local/lib/libopencv_imgproc.so \
               /usr/local/lib/libopencv_video.so
#x86_64-linux-gnu
