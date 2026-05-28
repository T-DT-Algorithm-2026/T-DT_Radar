#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gimbal_interface/msg/gimbal_angle.hpp>
#include <radio_interface/msg/buff.hpp>
#include <radio_interface/msg/fire.hpp>
#include <radio_interface/msg/hp.hpp>
#include <radio_interface/msg/password.hpp>
#include <radio_interface/msg/state.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <vision_interface/msg/radar2_sentry.hpp>
#include <vision_interface/msg/radar_warn.hpp>
#include <vision_interface/msg/sentry2_radar.hpp>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

void on_exit([[maybe_unused]] int sig) {
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions & options)
        : Node("rosbag_player_node", options) {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->get_parameter("rosbag_file", rosbag_file);

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera1/image", rclcpp::SensorDataQoS());
        image2_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera2/image", rclcpp::SensorDataQoS());
        match_info_publisher_ = this->create_publisher<vision_interface::msg::MatchInfo>("/match_info", 10);
        sentry2radar_publisher_ = this->create_publisher<vision_interface::msg::Sentry2Radar>("/sentry2RadarData", 10);
        radar2sentry_publisher_ = this->create_publisher<vision_interface::msg::Radar2Sentry>("/Radar2Sentry", 10);
        gimbal_usart_data_publisher_ = this->create_publisher<gimbal_interface::msg::GimbalAngle>("/gimbalUsartData", 10);
        gimbal_pub_publisher_ = this->create_publisher<gimbal_interface::msg::GimbalAngle>("/GimbalPub", 10);
        lidar_detect_publisher_ = this->create_publisher<vision_interface::msg::RadarWarn>("/lidar_detect", 10);
        key_usart_sender_publisher_ = this->create_publisher<radio_interface::msg::Password>("/key_usart_sender", 10);
        radio_buff_publisher_ = this->create_publisher<radio_interface::msg::Buff>("/radio_buff", 10);
        radio_fire_publisher_ = this->create_publisher<radio_interface::msg::Fire>("/radio_fire", 10);
        radio_hp_publisher_ = this->create_publisher<radio_interface::msg::Hp>("/radio_hp", 10);
        radio_state_publisher_ = this->create_publisher<radio_interface::msg::State>("/radio_state", 10);
        // 订阅控制话题，用于 pause/resume/toggle
        control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "rosbag_player/control", 10,
            std::bind(&RosbagPlayer::control_callback, this, std::placeholders::_1));
        signal(SIGINT, on_exit);
        // 创建一个新的线程来处理bag文件
        reader_.open(rosbag_file);

        processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer() {
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

private:
    void play_bag() {
        while (rclcpp::ok()) {
            // 如果处于暂停状态，则等待直到恢复
            if (paused_.load()) {
                std::unique_lock<std::mutex> lk(pause_mutex_);
                pause_cv_.wait(lk, [this]() { return !paused_.load(); });
            }

            if(!reader_.has_next())
                reader_.open(rosbag_file);

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();  
            
            // 处理 PointCloud2 消息
            if (bag_message->topic_name == "/livox/lidar") {
                auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }
            // 处理 CompressedImage 消息
            else if (bag_message->topic_name == "/compressed_image") {
                publish_compressed_image(bag_message, image_publisher_, ros_time);
            }
            // 处理第二路 CompressedImage 消息
            else if (bag_message->topic_name == "/compressed_image2") {
                publish_compressed_image(bag_message, image2_publisher_, ros_time);
            }
            // 处理match_info消息
            else if (bag_message->topic_name == "/match_info") {
                // RCLCPP_INFO(this->get_logger(), "Received match_info message");
                auto match_info_msg = std::make_shared<vision_interface::msg::MatchInfo>();
                rclcpp::Serialization<vision_interface::msg::MatchInfo> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                // std::cout<<"serialized_msg.size():"<<serialized_msg.size()<<std::endl;
                serialization.deserialize_message(&serialized_msg, match_info_msg.get());
                match_info_publisher_->publish(*match_info_msg);
            }
            else if (bag_message->topic_name == "/sentry2RadarData") {
                publish_serialized_message<vision_interface::msg::Sentry2Radar>(
                    bag_message, sentry2radar_publisher_);
            }
            else if (bag_message->topic_name == "/Radar2Sentry") {
                publish_serialized_message<vision_interface::msg::Radar2Sentry>(
                    bag_message, radar2sentry_publisher_);
            }
            else if (bag_message->topic_name == "/gimbalUsartData") {
                publish_gimbal_message(bag_message, gimbal_usart_data_publisher_, ros_time);
            }
            else if (bag_message->topic_name == "/GimbalPub") {
                publish_gimbal_message(bag_message, gimbal_pub_publisher_, ros_time);
            }
            else if (bag_message->topic_name == "/lidar_detect") {
                publish_serialized_message<vision_interface::msg::RadarWarn>(
                    bag_message, lidar_detect_publisher_);
            }
            else if (bag_message->topic_name == "/key_usart_sender") {
                publish_serialized_message<radio_interface::msg::Password>(
                    bag_message, key_usart_sender_publisher_);
            }
            else if (bag_message->topic_name == "/radio_buff") {
                publish_serialized_message<radio_interface::msg::Buff>(
                    bag_message, radio_buff_publisher_);
            }
            else if (bag_message->topic_name == "/radio_fire") {
                publish_serialized_message<radio_interface::msg::Fire>(
                    bag_message, radio_fire_publisher_);
            }
            else if (bag_message->topic_name == "/radio_hp") {
                publish_serialized_message<radio_interface::msg::Hp>(
                    bag_message, radio_hp_publisher_);
            }
            else if (bag_message->topic_name == "/radio_state") {
                publish_serialized_message<radio_interface::msg::State>(
                    bag_message, radio_state_publisher_);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // RCLCPP_INFO(this->get_logger(), "Processed one message, duration: %d ms", duration);
            if((duration < 10)&&(duration > 1)) {
                std::this_thread::sleep_for(10ms - std::chrono::milliseconds(duration));
            }
        }
        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }

    void publish_compressed_image(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher,
        const rclcpp::Time &stamp) {
        auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&serialized_msg, image_msg.get());

        auto img = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image from topic %s",
                        bag_message->topic_name.c_str());
            return;
        }

        auto header = image_msg->header;
        header.stamp = stamp;
        auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        publisher->publish(*msg);
    }

    template <typename MessageT>
    void publish_serialized_message(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message,
        const typename rclcpp::Publisher<MessageT>::SharedPtr &publisher) {
        auto msg = std::make_shared<MessageT>();
        rclcpp::Serialization<MessageT> serialization;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&serialized_msg, msg.get());
        publisher->publish(*msg);
    }

    void publish_gimbal_message(
        const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &bag_message,
        const rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr &publisher,
        const rclcpp::Time &stamp) {
        auto msg = std::make_shared<gimbal_interface::msg::GimbalAngle>();
        rclcpp::Serialization<gimbal_interface::msg::GimbalAngle> serialization;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&serialized_msg, msg.get());
        msg->header.stamp = stamp;
        publisher->publish(*msg);
    }

    void control_callback(const std_msgs::msg::String::SharedPtr msg) {
        const std::string &cmd = msg->data;
        if (cmd == "pause") {
            paused_.store(true);
            RCLCPP_INFO(this->get_logger(), "rosbag_player: paused");
        } else if (cmd == "resume") {
            paused_.store(false);
            pause_cv_.notify_all();
            RCLCPP_INFO(this->get_logger(), "rosbag_player: resumed");
        } else if (cmd == "toggle") {
            bool now = !paused_.load();
            paused_.store(now);
            if (!now) {
                pause_cv_.notify_all();
            }
            RCLCPP_INFO(this->get_logger(), "rosbag_player: toggled to %s", now ? "paused" : "running");
        } else {
            RCLCPP_INFO(this->get_logger(), "rosbag_player: unknown control '%s'", cmd.c_str());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image2_publisher_;
    rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr match_info_publisher_;
    rclcpp::Publisher<vision_interface::msg::Sentry2Radar>::SharedPtr sentry2radar_publisher_;
    rclcpp::Publisher<vision_interface::msg::Radar2Sentry>::SharedPtr radar2sentry_publisher_;
    rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_usart_data_publisher_;
    rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_pub_publisher_;
    rclcpp::Publisher<vision_interface::msg::RadarWarn>::SharedPtr lidar_detect_publisher_;
    rclcpp::Publisher<radio_interface::msg::Password>::SharedPtr key_usart_sender_publisher_;
    rclcpp::Publisher<radio_interface::msg::Buff>::SharedPtr radio_buff_publisher_;
    rclcpp::Publisher<radio_interface::msg::Fire>::SharedPtr radio_fire_publisher_;
    rclcpp::Publisher<radio_interface::msg::Hp>::SharedPtr radio_hp_publisher_;
    rclcpp::Publisher<radio_interface::msg::State>::SharedPtr radio_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string rosbag_file;
    std::atomic<bool> paused_{false};
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;

};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
