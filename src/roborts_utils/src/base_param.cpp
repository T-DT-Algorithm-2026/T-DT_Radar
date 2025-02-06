/*
 * @Name: JsonParam
 * @Description: 读取参数程序
 * @Version: 1.0.0.2
 * @Author: 严俊涵
 * @Date: 2023-01-03 16:34
 */
#pragma GCC optimize(2)
#include "base_param.h"

#include <unistd.h>

#include "base_msg.h"
using namespace std;
using namespace cv;

std::map<std::string, JsonParam::JsonParam> LoadParam::params;
namespace JsonParam {
void JsonParam::Init(string path) {
  int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    TDT_FATAL("配置文件%s打开失败", path.c_str());
  }
  if ((flock(fd, LOCK_EX | LOCK_NB)) < 0) {
    TDT_FATAL("配置文件%s被锁定,这可能是由另一个已启动未关闭的实例造成的",
              path.c_str());
  } else {
    path_ = path;
    std::ifstream jsonFile(path_);
    if (!jsonFile.is_open()) {
      TDT_FATAL("配置文件%s打开失败", path_.c_str());
    } else {
      Json::Reader jsonReader;

      if (!jsonReader.parse(jsonFile, jsonValue, true)) {
        TDT_FATAL("Json配置文件%s序列化失败", path_.c_str());
      }
    }
    flock(fd, LOCK_UN);
    close(fd);
    jsonFile.close();
  }
}

template <>
void ReadTheParam<cv::Mat>(const Json::Value &node, cv::Mat &param) {
  static std::map<char, int> mat_type2int = {{'u', 0}, {'c', 1}, {'w', 2},
                                             {'s', 3}, {'i', 4}, {'f', 5},
                                             {'d', 6}, {'r', 7}};
  // static char mat_type2char[8] = {'u', 'c', 'w', 's', 'i', 'f', 'd',
  // 'r'};

  if (!node.isMember("type") || !node.isMember("rows") ||
      !node.isMember("cols") || !node.isMember("xdata")) {
    throw JsonException("cv::Mat参数格式错误");
    return;
  }
  unsigned int rows = node["rows"].asInt();
  unsigned int cols = node["cols"].asInt();
  if (node["xdata"].size() != rows || node["xdata"][0].size() != cols) {
    throw JsonException("cv::Mat参数格式错误");
    return;
  }
  int type = mat_type2int.find(node["type"].asString()[0])->second;
  param = cv::Mat(rows, cols, type);
  for (unsigned int i = 0; i < rows; i++) {
    for (unsigned int j = 0; j < cols; j++) {
      switch (type) {
        case 0:
          param.at<uchar>(i, j) = node["xdata"][i][j].asInt();
          break;
        case 1:
          param.at<char>(i, j) = node["xdata"][i][j].asInt();
          break;
        case 2:
          param.at<ushort>(i, j) = node["xdata"][i][j].asInt();
          break;
        case 3:
          param.at<short>(i, j) = node["xdata"][i][j].asInt();
          break;
        case 4:
          param.at<int>(i, j) = node["xdata"][i][j].asInt();
          break;
        case 5:
          param.at<float>(i, j) = node["xdata"][i][j].asFloat();
          break;
        case 6:
          param.at<double>(i, j) = node["xdata"][i][j].asDouble();
          break;
        case 7:
          // param.at<cv::Vec3b>(i, j) =
          // cv::Vec3b(node["xdata"][i][j][0].asInt(),
          // node["xdata"][i][j][1].asInt(),
          // node["xdata"][i][j][2].asInt());
          throw JsonException("cv::Mat参数格式错误");
          break;
        default:
          throw JsonException("cv::Mat参数格式错误");
          return;
      }
    }
  }
}

template <>
void WriteTheParam<cv::Mat>(Json::Value &node, const cv::Mat &param) {
  static std::map<char, int> mat_type2int = {{'u', 0}, {'c', 1}, {'w', 2},
                                             {'s', 3}, {'i', 4}, {'f', 5},
                                             {'d', 6}, {'r', 7}};
  static char mat_type2char[8] = {'u', 'c', 'w', 's', 'i', 'f', 'd', 'r'};

  if (param.empty()) {
    throw JsonException("cv::Mat为空");
    return;
  }
  Json::Value temp = std::string("") + mat_type2char[param.type()];
  node["type"].swapPayload(temp);
  temp = param.rows;
  node["rows"].swapPayload(temp);
  temp = param.cols;
  node["cols"].swapPayload(temp);
  Json::Value xdata;
  for (int i = 0; i < param.rows; i++) {
    for (int j = 0; j < param.cols; j++) {
      switch (param.type()) {
        case 0:
          xdata[i][j] = param.at<uchar>(i, j);
          break;
        case 1:
          xdata[i][j] = param.at<char>(i, j);
          break;
        case 2:
          xdata[i][j] = param.at<ushort>(i, j);
          break;
        case 3:
          xdata[i][j] = param.at<short>(i, j);
          break;
        case 4:
          xdata[i][j] = param.at<int>(i, j);
          break;
        case 5:
          xdata[i][j] = param.at<float>(i, j);
          break;
        case 6:
          xdata[i][j] = param.at<double>(i, j);
          break;
        case 7:
          // xdata[i][j] = param.at<cv::Vec2d>(i, j);
          throw JsonException("该Mat类型的读取与写入尚未实现");
          break;
        default:
          throw JsonException("Mat类型错误");
          break;
      }
    }
  }
  node["xdata"].swapPayload(xdata);
}

void JsonParam::OutPutParam() {
  TDT_INFO("Saving Parameter...");
  auto now = std::chrono::system_clock::now();
  auto itt = std::chrono::system_clock::to_time_t(now);
  std::tm* tm = std::localtime(&itt);
  char date[80];
  strftime(date, sizeof(date),"%Y-%m-%d %H:%M:%S", tm);
  WriteTheParam(jsonValue["Date"], std::string(date));
  Json::StreamWriterBuilder jsonWriterBuilder;
  std::unique_ptr<Json::StreamWriter> jsonWriter(
      jsonWriterBuilder.newStreamWriter());
  std::ofstream jsonFile(path_);
  int fd = open(path_.c_str(), O_WRONLY);
  if (fd < 0) {
    TDT_ERROR("配置文件%s打开失败", path_.c_str());
  }
  if ((flock(fd, LOCK_EX | LOCK_NB)) < 0) {
    TDT_ERROR("配置文件%s被锁定,这可能是由另一个已启动未关闭的实例造成的",
              path_.c_str());
  }
  if (!jsonFile.is_open()) {
    TDT_ERROR("配置文件%s打开失败", path_.c_str());
  } else {
    jsonWriter->write(jsonValue, &jsonFile);
    jsonFile.close();
    TDT_INFO("参数保存(%s).", path_.c_str());
  }
}

Json::Value &JsonParam::FindRootValue(const std::string &check_name) {
  if (jsonValue.isMember(check_name)) {
    return jsonValue[check_name];
  } else if (jsonValue["DynamicParams"].isMember(check_name)) {
    return jsonValue["DynamicParams"][check_name];
  } else if (jsonValue["StaticParams"].isMember(check_name)) {
    return jsonValue["StaticParams"][check_name];
  } else {
    throw JsonException("未找到参数" + check_name);
  }
}
}  // namespace JsonParam