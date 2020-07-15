#include <QCoreApplication>
#include "gtest/gtest.h"
#include "modules/common/log/glogutil.h"
#include "modules/framework/conf/parameter.h"
#include "modules/framework/data/data.h"
#include "modules/framework/process/process.h"
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    std::string conf_file = "../modules/framework/conf/main.conf";
    std::string log_path = "log";
    lidar_algorithm::common::log::GlogUtil::get_obj()->init(argv[0],log_path);
    lidar_algorithm::framework::conf::Parameter::get_obj()->init(conf_file.c_str());
    if (lidar_algorithm::framework::conf::Parameter::get_obj()->_use_gtest) {
        testing::InitGoogleTest(&argc, argv);
        if (RUN_ALL_TESTS() == 0) {
            LOG(INFO)<<"all tests passed!";
        }else{
            LOG(WARNING)<<"some tests failed!";
        }
    }else{
        lidar_algorithm::framework::data::Data::get_obj()->process();
        lidar_algorithm::framework::process::Process::get_obj()->process();
    }
    return a.exec();
}
