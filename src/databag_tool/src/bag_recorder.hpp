/**
* @created  cheng
* @brief  订阅用户指定的多个话题并将它们记录到不同的rosbag文件中,从 yaml 配置中读取话题列表，然后在每个话题上创建订阅以捕获和记录数据
**/
#ifndef ROS_TWO_BAG_RECORDER_HPP
#define ROS_TWO_BAG_RECORDER_HPP


#include <iostream>
#include <chrono>
#include <sys/statfs.h>
#include "rosbag2_cpp/writer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <filesystem>
#include <rosbag2_cpp/converter_options.hpp>

class BagRecoder 
{
public:
    BagRecoder(std::shared_ptr<rclcpp::Node> &node_, const YAML::Node& config_record)
              : ros2_node_(node_)
    {   
        subscriptions = std::vector<rclcpp::SubscriptionBase::SharedPtr>(); // 重新初始化

        record_sections = std::vector<RecordSection>();

        count_of_bags = 1;
        record_section_num = 0;
        unfind_topic_num = 0;
        std::string relative_folder_path;
        try {
            relative_folder_path = config_record["bags_relative_path"].as<std::string>();
        } catch (YAML::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "获取relative_folder_path参数时出现错误，可能参数类型或者不存在该参数， config: %s", e.what());
            RCLCPP_WARN(node_->get_logger(),"将赋值设定参数,这是非正常的操作，即使正常运行，请检查修复");
            relative_folder_path = "../../../ros2bags/";
        }

        try {
            forced_stop_time = config_record["forced_stop_time"].as<int>();
        } catch (YAML::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "获取forced_stop_time参数时出现错误，可能参数类型或者不存在该参数， config: %s", e.what());
            RCLCPP_WARN(node_->get_logger(),"将赋值设定参数,这是非正常的操作，即使正常运行，请检查修复");
            forced_stop_time = 1000;
        }


        for (YAML::const_iterator it = config_record.begin(); it != config_record.end(); ++it) 
        {   // 首先遍历YAML文件的顶层节点
          std::string key = it->first.as<std::string>();
          // Check if key ends with "_bag_record"
          if (key.size() > 7 && key.substr(key.size() - 7) == "_record") 
          { 
            RecordSection section;
            section.unfind_topics = std::vector<std::string>();
            section.topic_info_of_this_section = std::vector<BagRecoder::TopicInfo>();

            for (const auto& topic : it->second) 
            {
              // section.topics_of_this_section.push_back(topic.as<std::string>());
              BagRecoder::TopicInfo topic_info = {topic.as<std::string>(),""};
              section.topic_info_of_this_section.push_back(topic_info);
              section.unfind_topics.push_back(topic.as<std::string>());
            }
            
            if(!section.unfind_topics.empty()) 
            {   // 如果这一个 record 单元是有话题的，才算入 record_section_num, unfind_topic_num, folder_paths_
                record_section_num++;
                // Print topics
                std::cout << "Topics for end: " << key << ":" << std::endl;
                for (const auto& topic : section.unfind_topics) 
                {
                    std::cout << "  - " << topic << "  ";
                }
                section.folder_path = std::string(ROOT_DIR) + relative_folder_path + key + generate_str_of_timestamp();
                std::cout << "are to saved in "<< relative_folder_path + key + generate_str_of_timestamp() << std::endl;
                if(std::filesystem::exists(section.folder_path)) // 如果存在，像 NUC、agx可能的时间混乱
                {   
                    std::cout<<"设备时间可能有些问题，出现重复的数据集文件夹，将妥善处理,但需要查一查";
                    int follow = 2;
                    while(true)
                    {
                      std::string temp_path = section.folder_path;
                      temp_path += "_"+std::to_string(follow);
                      if(!std::filesystem::exists(section.folder_path) )
                          break;
                      else follow++;
                    }

                }


                section.writer = std::make_unique<rosbag2_cpp::Writer>();
                
                // Add the new RecordSection instance to the record_sections vector
                record_sections.emplace_back(std::move(section));                
            }
            else{
                  std::cout << "No topics found for end: " << key << std::endl;
                  // The section object will be destroyed automatically in the next iteration of the loop
                }
          }
        }

        record_section_num = record_sections.size();
        
        for(int i=0; i<record_section_num; i++)
        {
          // 不需要检查 if the topic already in topics_of_each_sections, 
          // 因为即使有话题重复，也都是来自不同录制 section 的，不能排除，
          
          unfind_topic_num += record_sections.at(i).unfind_topics.size();

          // 创建文件夹
          std::filesystem::path bag_directory = record_sections.at(i).folder_path;
          if (!std::filesystem::exists(bag_directory))
          {   
              // 尝试创建指定的 bag_directory，如果该目录及其父目录（如 "ros2bags"）不存在，将被创建
              std::filesystem::create_directories(bag_directory);
          }
        }

    }

    void work();

    struct TopicInfo{
      std::string topic_name;
      std::string topic_type;
    };

    struct RecordSection{
      std::string folder_path;
      std::vector<TopicInfo> topic_info_of_this_section;
      std::unique_ptr<rosbag2_cpp::Writer> writer;
      std::vector<std::string> unfind_topics;

    };
    

private:
    void writer_create_topics(std::unique_ptr<rosbag2_cpp::Writer> &writer_, std::string topic_name, std::string topic_type);
    void search_topic(const std::string topic_name, std::unique_ptr<rosbag2_cpp::Writer> &writer_);
    void search_topic(RecordSection &section);
    long int get_available_disk_space_gb(const std::string& path);

    std::string generate_str_of_timestamp();
    void open_new_bag(std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &bag_path);

    std::vector<std::vector<std::string>> topics_of_each_sections;// 二维向量，包含每个记录部分的话题列表

    std::vector<RecordSection> record_sections;

    int record_section_num;
    
    std::shared_ptr<rclcpp::Node> ros2_node_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;

    int forced_stop_time;   // 强制停止时间,超过则录制会停止
    int count_of_bags ;
    int unfind_topic_num;
    std::mutex writer_mutex_; // 互斥锁，用于保护对rosbag2_cpp::Writer实例的访问
};


void BagRecoder::search_topic(const std::string topic_name, std::unique_ptr<rosbag2_cpp::Writer> &writer_)
{   
    // 获取 ROS 图中的话题信息
    auto topic_info_map = ros2_node_->get_topic_names_and_types();

    // 查找指定话题的类型
    std::string topic_type;
    for (const auto &topic_info : topic_info_map) 
    {
      if (topic_info.first == topic_name) 
      {
        topic_type = topic_info.second.front();
        break;
      }
    }
    // 如果找到了类型，则动态创建订阅 (创建一个泛型订阅器来订阅这个主题并处理接收到的消息)
    if (!topic_type.empty()) 
    {
      
      writer_create_topics(writer_, topic_name, topic_type);

      auto topics_interface = ros2_node_->get_node_topics_interface();
      rcutils_allocator_t allocator = rcutils_get_default_allocator();

      auto subscription = rclcpp::create_generic_subscription(
        topics_interface,
        topic_name,
        topic_type,
        12000,
        //rclcpp::SensorDataQoS(),
        [this, &topic_name, &allocator, &writer_](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          // 在这里处理消息,将消息写入 rosbag
          auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
          bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
          //         bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          // new rcutils_uint8_array_t,
          // [](rcutils_uint8_array_t *array) {
          //     rcutils_uint8_array_fini(array);
          //     delete array;
          // });
          bag_msg->topic_name = topic_name;
          bag_msg->recv_timestamp = ros2_node_->now().nanoseconds();
          bag_msg->serialized_data->buffer = msg->get_rcl_serialized_message().buffer;
          bag_msg->serialized_data->buffer_length = msg->get_rcl_serialized_message().buffer_length;
          bag_msg->serialized_data->buffer_capacity = msg->get_rcl_serialized_message().buffer_capacity;
          // bag_msg->serialized_data->allocator = msg->get_rcl_serialized_message().allocator;
          bag_msg->serialized_data->allocator = allocator;

          std::lock_guard<std::mutex> lock(writer_mutex_);

          writer_->write(bag_msg);
        });

        subscriptions.push_back(subscription);
        unfind_topic_num --;
        // RCLCPP_INFO(ros2_node_->get_logger(), "录制端捕获 topic %s of type %s", topic_name.c_str(), topic_type.c_str());
        // RCLCPP_INFO(ros2_node_->get_logger(), "unfind_topic_num %d", unfind_topic_num);

    } else {
      ;
      // RCLCPP_ERROR(ros2_node_->get_logger(), "Topic %s not found or has no type,注意是不是缺少/", topic_name.c_str());
    }
}

void BagRecoder::search_topic(RecordSection &section)
{   
    // 【改动1】：把获取 ROS 图信息的代码提到最前面，避免在循环里重复获取，提高效率
    auto topic_info_map = ros2_node_->get_topic_names_and_types();

    // 【改动2】：使用迭代器遍历所有尚未找到的话题
    for (auto it = section.unfind_topics.begin(); it != section.unfind_topics.end(); ) 
    {
        std::string topic_name = *it; // 获取当前遍历到的话题名称

        // 查找指定话题的类型
        std::string topic_type;
        for (const auto &topic_info : topic_info_map) 
        {
          if (topic_info.first == topic_name) 
          {
            topic_type = topic_info.second.front();
            break;
          }
        }
        
        // 如果找到了类型，则动态创建订阅 (创建一个泛型订阅器来订阅这个主题并处理接收到的消息)
        if (!topic_type.empty()) 
        {
          writer_create_topics(section.writer, topic_name, topic_type);

          auto topics_interface = ros2_node_->get_node_topics_interface();
          rcutils_allocator_t allocator = rcutils_get_default_allocator();

          auto subscription = rclcpp::create_generic_subscription(
            topics_interface,
            topic_name,
            topic_type,
            12000,
            //rclcpp::SensorDataQoS(),
            [this, topic_name, topic_type, allocator, &section](std::shared_ptr<rclcpp::SerializedMessage> msg) {
              // 在这里处理消息,将消息写入 rosbag
              auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
              bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
              bag_msg->topic_name = topic_name;
              bag_msg->recv_timestamp = ros2_node_->now().nanoseconds();
              bag_msg->serialized_data->buffer = msg->get_rcl_serialized_message().buffer;
              bag_msg->serialized_data->buffer_length = msg->get_rcl_serialized_message().buffer_length;
              bag_msg->serialized_data->buffer_capacity = msg->get_rcl_serialized_message().buffer_capacity;
              bag_msg->serialized_data->allocator = allocator;

              std::lock_guard<std::mutex> lock(writer_mutex_);

              section.writer->write(bag_msg);
              
            });

            subscriptions.push_back(subscription);
            unfind_topic_num --;
            for(auto& topic_info :section.topic_info_of_this_section)
            {
                if(topic_info.topic_name ==  topic_name)
                    topic_info.topic_type = topic_type;
            }
            
            // 【改动3】：使用迭代器删除找到的元素，并将迭代器自动指向下一个位置
            it = section.unfind_topics.erase(it);
            
            RCLCPP_INFO(ros2_node_->get_logger(), "录制端捕获 topic %s of type %s", topic_name.c_str(), topic_type.c_str());

        } else {
            // 【改动4】：如果没找到该话题，将迭代器加1，继续寻找下一个未找到的话题
            ++it;
        }
    }
}


void BagRecoder::writer_create_topics(std::unique_ptr<rosbag2_cpp::Writer> &writer_, std::string topic_name, std::string topic_type)
{   
    // std::lock_guard<std::mutex> lock(writer_mutex_);
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic_name;
    
    topic_metadata.type = topic_type;
    topic_metadata.serialization_format = "cdr";// Common Data Representation (CDR) 格式对 ROS 2 消息进行序列化

    writer_->create_topic(topic_metadata);
}

std::string BagRecoder::generate_str_of_timestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    char timestamp_str[100] = {0};
    std::strftime(timestamp_str, sizeof(timestamp_str), "%m%d_%H%M_%S", std::localtime(&now_time_t));

    return  timestamp_str;
}

void BagRecoder::open_new_bag(std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &bag_path)
{ 
    // std::lock_guard<std::mutex> lock(writer_mutex_);
    writer = std::make_unique<rosbag2_cpp::Writer>();
    
    rosbag2_storage::StorageOptions storage_options_new;
    storage_options_new.uri = bag_path;
    storage_options_new.storage_id = "sqlite3";
    writer->open(storage_options_new);
}

void BagRecoder::work() 
{   
    for(auto& section :record_sections )
    {
        open_new_bag(section.writer, section.folder_path+"/bag_"+generate_str_of_timestamp() );
    }

    std::thread([this]() {
        rclcpp::Rate rate10(10);
        while (unfind_topic_num) 
        { 

            // for(int i=0; i< record_section_num; i++)
            // { 
            //   // for (const auto &one_left_topic : topics_of_each_sections.at(i)) 
            //   // {
            //   //   search_topic(one_left_topic, record_sections.at(i).writer);
            //   // }
            //   for (auto &one_section : record_sections) 
            //   {
            //     search_topic(one_section);
            //   }

            // }
            for (auto &one_section : record_sections) 
            {
              search_topic(one_section);
            }
            rate10.sleep();
            
        }
        if(unfind_topic_num == 0)
        {
          std::cout<<"捕获所有目标 topic \n";
        }
        rclcpp::Rate rate0_1(0.1);
        while(rclcpp::ok()) //  !!!!!!!!
        {
          rate0_1.sleep();
        }
        rclcpp::shutdown();

    }).detach();

    std::thread([this]() {
      rclcpp::spin(ros2_node_);
      rclcpp::shutdown();
    }).detach();

    // 处理异常问题，每次录制在 bags 目录下新创文件夹，每隔三十秒保存一份，
    auto start_record_time = std::chrono::system_clock::now();
    auto start_bag_time = std::chrono::system_clock::now();
    rclcpp::Rate rate(10);
    while(rclcpp::ok())
    {
      if((std::chrono::system_clock::now() - start_record_time) > std::chrono::seconds(forced_stop_time))
        rclcpp::shutdown();

      // 检查磁盘剩余空间
      std::string path_to_check = "/"; // Use root directory for Linux
      long int available_disk_space_gb = get_available_disk_space_gb(path_to_check);
      // std::cout<<"available_disk_space_gb:"<<available_disk_space_gb<<"\n";
      if (available_disk_space_gb < 2) {
        std::cout<<"available_disk_space_gb:"<<available_disk_space_gb<<"\n";
        std::cerr << "磁盘空间不足12G，停止录制" << std::endl;
        rclcpp::shutdown();  // 停止录制
      }

          if ((std::chrono::system_clock::now() - start_bag_time) >= std::chrono::seconds(15))
          {
              // Open a new bag file
              count_of_bags++;
              for(auto& section :record_sections )
              {   
                  std::lock_guard<std::mutex> lock(writer_mutex_);
                  open_new_bag(section.writer, section.folder_path+"/bag_" + generate_str_of_timestamp() );
                  for(auto &topic_info : section.topic_info_of_this_section)
                  {   
                      writer_create_topics(section.writer, topic_info.topic_name, topic_info.topic_type);
                  }
              }
              // Reset the timer
              start_bag_time = std::chrono::system_clock::now();
          }

          rate.sleep();

    }

}


// 获取磁盘剩余空间（以GB为单位）
long int BagRecoder::get_available_disk_space_gb(const std::string& path) {
  struct statfs disk_info;
  statfs(path.c_str(), &disk_info);

  unsigned long long blocksize = disk_info.f_bsize;  // 每个block里包含的字节数
  unsigned long long available_disk = disk_info.f_bavail * blocksize;  // 可用空间大小
  long int true_available_disk_gb = available_disk >> 30;

  return true_available_disk_gb;
}


#endif // ROS_TWO_BAG_RECORDER_HPP
