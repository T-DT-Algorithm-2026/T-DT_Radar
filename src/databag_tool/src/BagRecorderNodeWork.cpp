#include "BagRecorderNode.hpp"




BagRecorderNode::BagRecorderNode(const rclcpp::NodeOptions & node_options)
    : Node("bag_recorder", node_options)
{   
    std::string yaml_config_path = std::string(ROOT_DIR) + "config_record.yaml";
    config_record = YAML::LoadFile(yaml_config_path);


    subscriptions = std::vector<rclcpp::SubscriptionBase::SharedPtr>(); // 重新初始化

    record_sections = std::vector<RecordSection>();

    count_of_bags = 1;
    record_section_num = 0;
    unfind_topic_num = 0;
    std::string relative_folder_path;
    try {
        relative_folder_path = config_record["bags_relative_path"].as<std::string>();
    } catch (YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "获取relative_folder_path参数时出现错误，可能参数类型或者不存在该参数， config: %s", e.what());
        RCLCPP_WARN(this->get_logger(),"将赋值设定参数,这是非正常的操作，即使正常运行，请检查修复");
        relative_folder_path = "../../../ros2bags/";
    }

    try {
        forced_stop_time = config_record["forced_stop_time"].as<int>();
    } catch (YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "获取forced_stop_time参数时出现错误，可能参数类型或者不存在该参数， config: %s", e.what());
        RCLCPP_WARN(this->get_logger(),"将赋值设定参数,这是非正常的操作，即使正常运行，请检查修复");
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
        section.topic_info_of_this_section = std::vector<BagRecorderNode::TopicInfo>();

        for (const auto& topic : it->second) 
        {
            // section.topics_of_this_section.push_back(topic.as<std::string>());
            BagRecorderNode::TopicInfo topic_info = {topic.as<std::string>(),""};
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

    main_work_thread_ = std::thread(&BagRecorderNode::work, this);


}

BagRecorderNode::~BagRecorderNode()
{
    stop_requested_ = true;

    if (main_work_thread_.joinable()) {
        main_work_thread_.join();
    }
    if (topic_search_thread_.joinable()) {
        topic_search_thread_.join();
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(BagRecorderNode)
