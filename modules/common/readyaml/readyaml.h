/**
* readyaml.h
* Author: zhubin
* Created on: 2019-05-28
* Copyright (c) iRotran. All Rights Reserved
*/
#ifndef READYAML_H
#define READYAML_H
#include <yaml-cpp/yaml.h>
#include <string>
#include "modules/common/common.h"
namespace lidar_algorithm {
namespace common {
namespace readyaml {
class ReadYaml
{
public:
    ReadYaml(){}
    ~ReadYaml(){}
public:
    //YAML::Node load_conf(const std::string& conf_file);
    YAML::Node load_conf(const std::string& param_file)
    {
        YAML::Node yaml_param = YAML::LoadFile(param_file);
        return yaml_param;
    }
    template<typename T>
    int parse_yaml(const YAML::Node yaml_node, const std::string& key, T& value, const T& default_value);
    template<typename T>
    int parse_yaml(const YAML::Node yaml_node, const std::string& key1, const std::string& key2,T& value, const T& default_value);
    template<typename T>
    int parse_yaml(const YAML::Node yaml_node, const std::string& key1, const std::string& key2,const std::string& key3,T& value, const T& default_value);
};
//ReadYaml::ReadYaml()
//{

//}
//ReadYaml::~ReadYaml()
//{

//}
//YAML::Node ReadYaml::load_conf(const std::string& param_file)
//{
//    YAML::Node yaml_param = YAML::LoadFile(param_file);
//    return yaml_param;
//}
template<typename T>
int ReadYaml::parse_yaml(const YAML::Node yaml_node, const std::string& key, T& value, const T& default_value)
{
    if (!yaml_node[key]){
        value = default_value;
        return RET_ERROR;
    }else{
        value = yaml_node[key].as<T>();
        return RET_OK;
    }
}
template<typename T>
int ReadYaml::parse_yaml(const YAML::Node yaml_node, const std::string& key1, const std::string& key2,T & value, const T & default_value)
{
    if (!yaml_node[key1][key2]) {
        value = default_value;
        return RET_ERROR;
    }else{
        value = yaml_node[key1][key2].as<T>();
        return RET_OK;
    }
}
template<typename T>
int ReadYaml::parse_yaml(const YAML::Node yaml_node, const std::string& key1, const std::string& key2,const std::string& key3,T& value, const T& default_value)
{
    if (!yaml_node[key1][key2][key3]) {
        value = default_value;
        return RET_ERROR;
    }else{
        value = yaml_node[key1][key2][key3].as<T>();
        return RET_OK;
    }
}
}//namespace readyaml
}//namespace common
}//namespace lidar_algorithm
#endif // READYAML_H
