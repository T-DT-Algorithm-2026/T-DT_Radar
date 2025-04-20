/**
* @created  cheng
* @brief  与 recorder 配套食用，yaml 配置数据集文件路径，在对应文件夹生成 merged_bag,并调用指令生成 index 文件
**/
#include <iostream>
#include <memory>
#include <chrono>
#include "rosbag2_cpp/writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <filesystem>
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"


using namespace std;
namespace fs = std::filesystem;

// 获取目录下的所有子目录
void get_subdirectories(const string& dir_path, vector<string>& subdirs) {
  subdirs.clear();
  for (auto& p : fs::directory_iterator(dir_path)) {
    if (p.is_directory()) {
      subdirs.push_back(p.path().string());
    }
  }
}

// 判断目录是否包含merged_bag
bool contains_merged_bag(const string& dir_path) {
  for (auto& p : fs::directory_iterator(dir_path)) {
    if (p.is_directory() && p.path().filename() == "merged_bag") {
      return true;
    }
  }
  return false;
}

// 合并一个目录下的所有bag文件
void merge_directory(const std::vector<std::string> &input_bag_files, const string& output_bag_path) 
{
    std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
      
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_bag_path;
    storage_options.storage_id = "sqlite3";
    writer->open(storage_options);

    for (const auto &input_bag_file : input_bag_files)
    {
        std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();

        // 打开输入bag文件
        rosbag2_storage::StorageOptions input_storage_options;
        input_storage_options.uri = input_bag_file;
        input_storage_options.storage_id = "sqlite3";

        try {
        reader->open(input_storage_options);
        } catch (const std::exception &e) {
          cerr << "Failed to open bag: " << input_bag_file << ". Reason: " << e.what() << endl;
          cout << "数据集将不merge该数据集及该数据集后的数据集，请检查" << endl;
          break; // 跳出循环
        }


        // 获取输入bag文件中的所有主题
        auto topics_and_types = reader->get_all_topics_and_types();

        // 在输出bag文件中创建输入 bag 文件的所有主题
        for (const auto &topic_and_type : topics_and_types)
        {
            writer->create_topic(topic_and_type);
        }

        // 读取并写入所有消息
        while (reader->has_next())
        {
            auto message = reader->read_next();
            writer->write(
                            message
                         );
        }

        // 关闭输入bag文件
        reader.release();
        
    }

  writer.release();

}

void create_index_yaml(const string& subdir, const string& output_bag_path)
{
    (void)subdir;
    // 添加以下代码以自动重新索引生成 metadata.yaml 文件
    std::string reindex_command = "ros2 bag reindex " + output_bag_path;
    int result = std::system(reindex_command.c_str());
    if (result == 0)
    {
      cout << "Metadata.yaml for " << output_bag_path << " has been generated." << endl;
    }
    else
    {
      cerr << "Failed to generate metadata.yaml for " << output_bag_path << ". Error code: " << result << endl;
    }
}

int main() {

  std::string yaml_config_path = string(ROOT_DIR) + "config_record.yaml";
  YAML::Node config_record = YAML::LoadFile(yaml_config_path);
  string bags_dir = std::string(ROOT_DIR) + config_record["bags_relative_path"].as<std::string>(); 
  if(bags_dir.back()=='/')
      bags_dir.pop_back();
  vector<string> subdirs;subdirs.clear();
  get_subdirectories(bags_dir, subdirs);

  for (auto& subdir : subdirs) 
  {
    vector<string> record_dirs;record_dirs.clear();
    get_subdirectories(subdir, record_dirs);
    // 添加以下代码以对 record_dirs 向量进行排序
    std::sort(record_dirs.begin(), record_dirs.end());
    
    if (!contains_merged_bag(subdir)) 
    {
      string output_bag_path = (fs::path(subdir) / "merged_bag").string();
      merge_directory(record_dirs, output_bag_path);
      cout << "合并数据集文件夹： " << subdir << " created at " << output_bag_path << endl;
      create_index_yaml(subdir, output_bag_path);

    }
  }

  return 0;
}