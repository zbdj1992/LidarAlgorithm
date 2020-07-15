/**
* read_text.h
* Author: zhubin
* Created on: 2020-06-17
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef READ_TEXT_H
#define READ_TEXT_H
#include <string>
#include <vector>
namespace lidar_algorithm {
namespace common {
namespace util{
class ReadText
{
public:
    ReadText();
    ~ReadText();
public:
    bool read_text_line(const std::string& file_path,
                   std::vector<std::string>& strs_vec);
};
}//namespace util
}//namespace common
}//namespace lidar_algorithm
#endif // READ_TEXT_H
