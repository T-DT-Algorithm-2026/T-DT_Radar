#ifndef RADAR_DETECT_FLY_H
#define RADAR_DETECT_FLY_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_interface/msg/fly_detect.hpp"
#include "vision_interface/msg/resolve_result.hpp"

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
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<vision_interface::msg::ResolveResult>::SharedPtr resolve_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr player_control_pub_;

    void createTrackbars();
    cv::Point2f grad_search(const cv::Mat& img, const cv::Point2f& center, const cv::Point2f& direction);
    KeyPoints pca_points(const std::vector<cv::Point>& contours, const cv::Mat& img);
    cv::Rect getSafeRect(cv::Mat& image, cv::Rect& rect);

private:
    // int h_min = 0, h_max = 95;
    // int s_min = 0, s_max = 125;
    int h_min = 57, h_max = 172;
    int s_min = 0, s_max = 191;
    int v_min = 200, v_max = 255;
    int g_min =0 , g_max =255;

    std::shared_ptr<Infer<yolo::BoxArray>> fly;
    std::string fly_path;

    double e=0;
    int de = 1;

    std::string save_dir_;
    bool save_images_{true};
    int image_save_counter_{0};
    std::chrono::steady_clock::time_point last_save_time_;

    // int id=0;
};
}  // namespace tdt_radar

#endif
