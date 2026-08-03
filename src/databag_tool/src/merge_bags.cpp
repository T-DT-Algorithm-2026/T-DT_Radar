/**
 * @created cheng
 * @brief 将分段录制的 SQLite3 bag 合并为带完整消息定义的 MCAP bag。
 */
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/message_definition.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
namespace fs = std::filesystem;

enum class MergeResult
{
    COMPLETE,
    PARTIAL,
    FAILED
};

void get_subdirectories(const string & dir_path, vector<string> & subdirs)
{
    subdirs.clear();
    for (const auto & path : fs::directory_iterator(dir_path))
    {
        if (path.is_directory())
        {
            subdirs.push_back(path.path().string());
        }
    }
}

bool contains_merged_bag(const string & dir_path)
{
    return fs::is_directory(fs::path(dir_path) / "merged_bag");
}

rosbag2_storage::MessageDefinition find_message_definition(
    const string & topic_type,
    const vector<rosbag2_storage::MessageDefinition> & input_definitions,
    rosbag2_cpp::LocalMessageDefinitionSource & local_definition_source)
{
    for (const auto & definition : input_definitions)
    {
        if (definition.topic_type == topic_type && !definition.encoded_message_definition.empty())
        {
            return definition;
        }
    }

    // 兼容未保存 schema 的旧 bag，从本机已安装的接口包补齐自定义消息及其依赖。
    try
    {
        return local_definition_source.get_full_text(topic_type);
    }
    catch (const std::exception & error)
    {
        cerr << "无法获取消息定义 " << topic_type << ": " << error.what() << endl;
        return rosbag2_storage::MessageDefinition::empty_message_definition_for(topic_type);
    }
}

MergeResult merge_directory(const vector<string> & input_bag_paths, const string & output_bag_path)
{
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions output_options;
    output_options.uri = output_bag_path;
    output_options.storage_id = "mcap";

    try
    {
        writer.open(output_options);
    }
    catch (const std::exception & error)
    {
        cerr << "无法创建 MCAP bag: " << output_bag_path << "，原因: " << error.what() << endl;
        return MergeResult::FAILED;
    }

    unordered_map<string, string> created_topics;
    rosbag2_cpp::LocalMessageDefinitionSource local_definition_source;
    bool all_inputs_merged = true;

    for (const auto & input_bag_path : input_bag_paths)
    {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions input_options;
        input_options.uri = input_bag_path;
        input_options.storage_id = "sqlite3";

        try
        {
            reader.open(input_options);
        }
        catch (const std::exception & error)
        {
            cerr << "无法打开 bag: " << input_bag_path << "，原因: " << error.what() << endl;
            cerr << "该数据集及其后的数据集不会被合并，请检查。" << endl;
            all_inputs_merged = false;
            break;
        }

        vector<rosbag2_storage::MessageDefinition> input_definitions;
        try
        {
            reader.get_all_message_definitions(input_definitions);
        }
        catch (const std::exception & error)
        {
            cerr << "bag 中没有可读取的消息定义，将从本机接口包补齐: " << error.what() << endl;
        }

        const auto topics_and_types = reader.get_all_topics_and_types();
        for (const auto & topic : topics_and_types)
        {
            const auto created_topic = created_topics.find(topic.name);
            if (created_topic != created_topics.end())
            {
                if (created_topic->second != topic.type)
                {
                    cerr << "同一话题存在不同消息类型，停止合并: " << topic.name << endl;
                    all_inputs_merged = false;
                }
                continue;
            }

            const auto definition = find_message_definition(topic.type, input_definitions, local_definition_source);
            if (definition.encoded_message_definition.empty())
            {
                cerr << "话题 " << topic.name << " 缺少消息定义，生成的 MCAP 无法独立解析该话题。" << endl;
            }
            writer.create_topic(topic, definition);
            created_topics.emplace(topic.name, topic.type);
        }

        if (!all_inputs_merged)
        {
            break;
        }

        while (reader.has_next())
        {
            writer.write(reader.read_next());
        }
        reader.close();
    }

    // 正常关闭 writer 会同时落盘 MCAP footer 和 metadata.yaml，无需再执行 reindex。
    writer.close();
    return all_inputs_merged ? MergeResult::COMPLETE : MergeResult::PARTIAL;
}

int main()
{
    const string yaml_config_path = string(ROOT_DIR) + "config_record.yaml";
    const YAML::Node config_record = YAML::LoadFile(yaml_config_path);
    string bags_dir = string(ROOT_DIR) + config_record["bags_relative_path"].as<string>();
    if (!bags_dir.empty() && bags_dir.back() == '/')
    {
        bags_dir.pop_back();
    }

    vector<string> subdirs;
    get_subdirectories(bags_dir, subdirs);
    sort(subdirs.begin(), subdirs.end());

    for (const auto & subdir : subdirs)
    {
        if (contains_merged_bag(subdir))
        {
            cout << "跳过已合并的数据集: " << subdir << endl;
            continue;
        }

        vector<string> record_dirs;
        get_subdirectories(subdir, record_dirs);
        sort(record_dirs.begin(), record_dirs.end());
        if (record_dirs.empty())
        {
            continue;
        }

        const string output_bag_path = (fs::path(subdir) / "merged_bag").string();
        const MergeResult result = merge_directory(record_dirs, output_bag_path);
        if (result == MergeResult::FAILED)
        {
            continue;
        }

        cout << "MCAP 数据集已生成: " << output_bag_path;
        if (result == MergeResult::PARTIAL)
        {
            cout << "（仅包含损坏位置之前的有效 bag）";
        }
        cout << endl;
    }

    return 0;
}
