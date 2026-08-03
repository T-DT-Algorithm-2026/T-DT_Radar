#ifndef RADAR_DETECT_FLY_H
#define RADAR_DETECT_FLY_H

#include <iostream>
#include <array>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_interface/msg/fly_detect.hpp"
#include "vision_interface/msg/resolve_result.hpp"
#include "vision_interface/msg/detect_fly.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "yolos.hpp"
#include "NvidiaInterface.hpp"
#include <fstream>
#include "classify.hpp"
#include "BaseInfer.hpp"
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace tdt_radar {

struct KeyPoints 
{
    cv::Point2f center;      // 矩形中心
    cv::Point2f left_mid;    // 左侧短边中点
    cv::Point2f right_mid;   // 右侧短边中点
};

class DetectFly final : public rclcpp::Node {
public:
    explicit DetectFly(const rclcpp::NodeOptions& options);
    ~DetectFly() override;
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;//相机图片
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player_control_pub_;//暂停
    rclcpp::Publisher<vision_interface::msg::DetectFly>::SharedPtr fly_pub_;//检测点

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_pub_;//压缩图片

    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr lidar_sub;
    void lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg);

    cv::Rect getSafeRect(cv::Mat& image, cv::Rect& rect);
    bool getLineCenter(const std::vector<cv::Point2f>& pts, cv::Point2f& center);

private:
    struct CompressTask
    {
        sensor_msgs::msg::Image::SharedPtr image;
        bool has_target = false;
        cv::Rect target_rect;
        cv::Point2f detect_point;
        cv::Point2f aim_point;
    };

    void scheduleCompress(CompressTask task);
    void compressLoop();
    void compressAndPublish(const CompressTask& task);

    std::shared_ptr<Infer<yolo::BoxArray>> fly;
    std::string fly_path;

    cv::Point2f target_point;
    rclcpp::Time lidar_time;
    float A_x = 0.0f, B_x = 845.0f;
    float A_y = 0.0f, B_y = 500.0f;

    std::string save_dir_ = "./saved_images";
    int image_save_counter_{0};
    bool save_images_ = false;

    int compress_image_ = 0;
    int draw_compressed_image_ = 0;
    bool has_compressed_subscriber_ = false;
    int compressed_image_quality_ = 50;
    std::mutex compress_mutex_;
    std::condition_variable compress_cv_;
    std::optional<CompressTask> pending_compress_task_;
    std::thread compress_thread_;
    std::atomic<bool> compress_running_{false};
    std::atomic<uint64_t> dropped_compress_frames_{0};
    std::chrono::steady_clock::time_point last_save_time_;
    std::chrono::steady_clock::time_point last_subscriber_check_time_{};
};
}  // namespace tdt_radar

#endif
