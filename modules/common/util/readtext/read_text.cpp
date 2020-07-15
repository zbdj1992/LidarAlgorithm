/**
* read_text.cpp
* Author: zhubin
* Created on: 2020-06-17
* Copyright (c) iRotran. All Rights Reserved
*/
#include "read_text.h"
#include <fstream>
namespace lidar_algorithm {
namespace common {
namespace util{
ReadText::ReadText()
{

}
ReadText::~ReadText()
{

}
bool ReadText::read_text_line(const std::string& file_path,
               std::vector<std::string>& strs_vec)
{
    std::ifstream is;
    is.open(file_path.data());
    if (!is.is_open()) {
        return false;
    }
    std::string str;
    strs_vec.clear();
    while (std::getline(is,str)) {
        strs_vec.push_back(str);
    }
    is.close();
    return true;
}
}//namespace util
}//namespace common
}//namespace lidar_algorithm
