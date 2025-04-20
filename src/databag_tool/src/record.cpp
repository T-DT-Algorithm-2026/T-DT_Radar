/**
* @created  cheng
* @brief  订阅用户指定的多个话题并将它们记录到不同的rosbag文件中,从 yaml 配置中读取话题列表，然后在每个话题上创建订阅以捕获和记录数据
**/
#include "bag_recorder.hpp"
using namespace std;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("normal_record_end");
    std::string yaml_config_path = string(ROOT_DIR) + "config_record.yaml";
    YAML::Node config_record = YAML::LoadFile(yaml_config_path);
    
    BagRecoder recorder(ros2_node, config_record);
    recorder.work();

    rclcpp::shutdown();
    return 0;
}

