/**
* read_text_test.cpp
* Author: zhubin
* Created on: 2020-06-17
* Copyright (c) iRotran. All Rights Reserved
*/
#include <gtest/gtest.h>
#include "read_text.h"
namespace lidar_algorithm {
namespace common {
namespace util{
TEST(ReadTextTest,read){
    std::string file_path = "../modules/common/util/readtext/pcd_name_list.txt";
    std::vector<std::string> strs_vec;
    ReadText readtext_obj;
    readtext_obj.read_text_line(file_path,strs_vec);
    EXPECT_EQ(strs_vec.size(),1805);
    EXPECT_STREQ(strs_vec[0].data(),"/home/zhubin/2020-06-16.16.18.52.981011/1592295533086.pcd");
    EXPECT_STREQ(strs_vec[1804].data(),"/home/zhubin/2020-06-16.16.18.52.981011/1592295756707.pcd");
}
}//namespace read_text
}//namespace common
}//namespace lidar_algorithm
