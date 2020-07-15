/**
* mutexmanager.h
* Author: zhubin
* Created on: 2019-05-25
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef MUTEXMANAGER_H
#define MUTEXMANAGER_H
#include <boost/thread/thread.hpp>
namespace lidar_algorithm {
namespace manager {
namespace mutex {
class MutexManager
{
public:
    MutexManager();
    ~MutexManager();
public:
    static MutexManager* get_obj() {
        static MutexManager mutexmanager_obj;
        return &mutexmanager_obj;
    }
public:
    boost::shared_mutex _lidar_data_shmutex;
    boost::shared_mutex _lidarlib_data_shmutex;
};
}//namespace mutex
}//namespace manager
}//namespace lidar_algorithm
#endif // MUTEXMANAGER_H
