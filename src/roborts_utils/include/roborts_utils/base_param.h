/*
 * @Name: LoadParam
 * @Description: 读取参数程序头文件,使用时直接包含此头文件，！！使用方法：
 * LoadParam::(函数名)
 * @Version: 1.0.2
 * @Author: 严俊涵
 * @Date: 2023-01-03 16:34
 * @Update: 2023-07-19 10:10
 */
#ifndef __BASE_PARAM_H
#define __BASE_PARAM_H

#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <sys/file.h>

#include <fstream>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#if __cplusplus >= 202002L
#include <experimental/source_location>  // TODO: 当你看到这里报错的时候，说明现在应该是不知道多久以后了，你开始使用C++23标准了，在完成该代码时使用了这个C++20的实验性功能，但不知道该实验性功能之后会怎样变化，所以请根据你当时的实际情况修改
#endif
#include <string>
#include <type_traits>
#include <vector>

#include "base_msg.h"
namespace JsonParam {
class JsonException : public std::exception {
 public:
  JsonException(const std::string &msg) : msg_(msg) {}
  virtual ~JsonException() throw() {}
  virtual const char *what() const throw() { return msg_.c_str(); }

 private:
  std::string msg_;
};

class JsonParam {
 public:
  void Init(std::string path);

  JsonParam() = default;
  ~JsonParam() = default;

  /**
   * @name OutPutParam
   * @brief 保存参数到文件
   */
  void OutPutParam();

  inline Json::Value &GetJsonValue() { return jsonValue; }

  template <typename... Args>
  Json::Value &GetJsonValue(std::string arg, Args... args) {
    auto &value = jsonValue;
    return GetJsonValueByValue(FindRootValue(arg), args...);
  }

  std::string GetPath() { return path_; }

 private:
  Json::Value jsonValue;
  std::string path_;

  Json::Value empty_node;

  Json::Value &FindRootValue(const std::string &check_name);

  Json::Value &GetJsonValueByValue(Json::Value &value) { return value; }

  template <typename... Args>
  Json::Value &GetJsonValueByValue(Json::Value &value, const std::string &arg,
                                   Args... args) {
    if (value[arg].isNull()) {
      throw JsonException(arg + "参数不存在");
    }
    return GetJsonValueByValue(value[arg], args...);
  }
};

/**
 * @name ReadTheParam
 * @brief 读取参数
 * @param [in] check_name 变量在config文件中的名字
 * @param [out] param 将结果保存在该变量中
 */
template <class T>
void ReadTheParam(const Json::Value &value, T &param) {
  try {
    param = value.as<T>();
  } catch (Json::Exception &e) {
    throw JsonException("参数类型不匹配");
  }
}

template <>
void ReadTheParam<cv::Mat>(const Json::Value &value, cv::Mat &param);

/**
 * @name WriteTheParam
 * @brief 更改程序中对应的参数
 * @param [in] check_name 变量在config文件中的名字
 * @param [in] param 变量的值
 */
template <class T>
void WriteTheParam(Json::Value &value, const T &param) {
  Json::Value temp = param;
  value.swapPayload(temp);  // 只有通过这种方法赋值才能保留注释
}

template <>
void WriteTheParam<cv::Mat>(Json::Value &value, const cv::Mat &param);

}  // namespace JsonParam

class LoadParam {
 public:
  struct ParamNameFmt {
    std::string value;
#if __cplusplus >= 202002L
    std::experimental::source_location loc;
    ParamNameFmt(const std::string &value,
                 const std::experimental::source_location &loc =
                     std::experimental::source_location::current()) {
      this->value = value;
      this->loc = loc;
    }

    ParamNameFmt(const char *value,
                 const std::experimental::source_location &loc =
                     std::experimental::source_location::current()) {
      this->value = value;
      this->loc = loc;
    }
#else
    ParamNameFmt(const std::string &value) { this->value = value; }
    ParamNameFmt(const char *value) { this->value = value; }
#endif
  };

 private:
  static std::map<std::string, JsonParam::JsonParam> params;

 public:
  inline static void InitParam(const std::string &param_name,
                               std::string path) {

    params[param_name] = JsonParam::JsonParam();
    params[param_name].Init(path);
  }

  inline static void InitROSParam(
      const std::string &param_name,
      const std::shared_ptr<rclcpp::Node> &ros_node) {
    try {
      auto dynamic_values = params[param_name].GetJsonValue("DynamicParams");
      for (auto &name : dynamic_values.getMemberNames()) {
        if (dynamic_values[name].isString()) {
          ros_node->declare_parameter(name, dynamic_values[name].asString());
        } else if (dynamic_values[name].isBool()) {
          ros_node->declare_parameter(name, dynamic_values[name].asBool());
        } else if (dynamic_values[name].isInt()) {
          ros_node->declare_parameter(name, dynamic_values[name].asInt());
        } else if (dynamic_values[name].isDouble()) {
          ros_node->declare_parameter(name, dynamic_values[name].asDouble());
        } else {
          TDT_ERROR("参数 %s 类型不支持", name.c_str());
        }
      }
    } catch (JsonParam::JsonException &e) {
      TDT_ERROR("在读取配置文件 %s(%s) 中出现错误: %s", param_name.c_str(),
                params[param_name].GetPath().c_str(), e.what());
    }
  }

  template <class... Args, class T>
  inline static void ReadParam(ParamNameFmt param_name,
                               const std::string &check_name, Args... args,
                               T &param) {
    // printf("param_address: %p\n", &params);

    auto param_file = params.find(param_name.value);
    if (param_file == params.end()) {
#if __cplusplus >= 202002L
      Logger(FATAL).taken(param_name.loc.line(), param_name.loc.file_name(),
                          "未找到配置文件 %s", param_name.value.c_str());
#else
      TDT_ERROR("未找到配置文件 %s", param_name.value.c_str());
#endif
    }
    try {
      auto &value = params[param_name.value].GetJsonValue(check_name, args...);
      JsonParam::ReadTheParam(value, param);
    } catch (JsonParam::JsonException &e) {
#if __cplusplus >= 202002L
      Logger(FATAL).taken(param_name.loc.line(), param_name.loc.file_name(),
                          "在读取配置文件 %s(%s) 中出现错误: %s",
                          param_name.value.c_str(),
                          params[param_name.value].GetPath().c_str(), e.what());
#else
      TDT_ERROR("在读取配置文件 %s(%s) 中出现错误: %s",
                param_name.value.c_str(),
                params[param_name.value].GetPath().c_str(), e.what());
#endif
    }
  }

  template <class... Args>
  inline static bool ExistParam(const std::string &param_name,
                                const std::string &check_name, Args... args) {
    try {
      auto &value = params[param_name].GetJsonValue(check_name, args...);
      return true;
    } catch (JsonParam::JsonException &e) {
      return false;
    }
  }

  template <class... Args, class T>
  inline static void WriteParam(ParamNameFmt param_name, const T &param,
                                const std::string &check_name, Args... args) {
    auto param_file = params.find(param_name.value);
    if (param_file == params.end()) {
#if __cplusplus >= 202002L
      Logger(FATAL).taken(param_name.loc.line(), param_name.loc.file_name(),
                          "未找到配置文件 %s", param_name.value.c_str());
#else
      TDT_ERROR("未找到配置文件 %s", param_name.value.c_str());
#endif
    }
    try {
      auto &value = params[param_name.value].GetJsonValue(check_name, args...);
      JsonParam::WriteTheParam(value, param);
    } catch (JsonParam::JsonException &e) {
#if __cplusplus >= 202002L
      Logger(FATAL).taken(param_name.loc.line(), param_name.loc.file_name(),
                          "在写入配置文件 %s(%s) 中出现错误: %s",
                          param_name.value.c_str(),
                          params[param_name.value].GetPath().c_str(), e.what());
#else
      TDT_ERROR("在写入配置文件 %s(%s) 中出现错误: %s",
                param_name.value.c_str(),
                params[param_name.value].GetPath().c_str(), e.what());
#endif
    }
  }
  inline static void OutPutParam(const std::string &param_name) {
    params[param_name].OutPutParam();
  }

  inline static void OutPutROSParam(
      const std::string &param_name,
      const std::shared_ptr<rclcpp::Node> &ros_node) {
    auto dynamic_values = params[param_name].GetJsonValue("DynamicParams");
    for (auto &name : dynamic_values.getMemberNames()) {
      auto ros_param = ros_node->get_parameter(name);
      if (dynamic_values[name].isString()) {
        LoadParam::WriteParam(param_name, ros_param.get_value<std::string>(),
                              name);
      } else if (dynamic_values[name].isBool()) {
        LoadParam::WriteParam(param_name, ros_param.get_value<bool>(), name);
      } else if (dynamic_values[name].isInt()) {
        LoadParam::WriteParam(param_name, ros_param.get_value<int>(), name);
      } else if (dynamic_values[name].isDouble()) {
        LoadParam::WriteParam(param_name, ros_param.get_value<double>(), name);
      } else {
        TDT_ERROR("参数 %s 类型不支持", name.c_str());
      }
    }
    params[param_name].OutPutParam();
  }

  inline static void OutPutAllParam() {
    for (auto &param : params) {
      param.second.OutPutParam();
    }
  }

  inline static int Size() { return params.size(); }

  /****
   * @brief 通过字符串数据更新参数
   * @param [in] param_name 参数命名空间
   * @param [in] check_name 参数名
   * @param [in] type 参数类型
   * @param [out] value 字符串参数值
   */
  template <class... Args>
  static void WriteParamString(ParamNameFmt param_name,
                               const std::string &value,
                               const std::string &check_name, Args... args,
                               const std::string &type) {
    if (type == "int") {
      int value_int = std::stoi(value);
      WriteParam(param_name, value_int, check_name, args...);
    } else if (type == "double") {
      float value_float = std::stof(value);
      WriteParam(param_name, value_float, check_name, args...);
    } else if (type == "float") {
      double value_double = std::stod(value);
      WriteParam(param_name, value_double, check_name, args...);
    } else if (type == "bool") {
      bool value_bool = (bool)std::stoi(value);
      WriteParam(param_name, value_bool, check_name, args...);
    } else if (type == "Mat") {
    } else {
      TDT_ERROR("未知的参数类型");
    }
  }

  static Json::Value GetJsonValue(const std::string &param_name) {
    return params[param_name].GetJsonValue();
  }
};  // namespace LoadParam

#endif  // _TDT__LOAD_PARAM_H