#ifndef RADAR_DETECT_H
#define RADAR_DETECT_H

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include "classify.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "NvidiaInterface.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_interface/msg/detect_result.hpp"
#include "vision_interface/msg/fly_points.hpp"
#include "yolos.hpp"
#include "BaseInfer.hpp"
#include <fstream>

#include <pcl/point_cloud.h>            // 用于 pcl::PointCloud 类型
#include <pcl/point_types.h>            // 用于 pcl::PointXYZ 类型
#include <pcl_conversions/pcl_conversions.h>
 
namespace tdt_radar {

class Detect final : public rclcpp::Node {
public:
    explicit Detect(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    void fly_callback(const std::shared_ptr<vision_interface::msg::FlyPoints> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;
    rclcpp::Subscription<vision_interface::msg::FlyPoints>::SharedPtr fly_sub;
    cv::Point3f fly_point;
    cv::Point2f get_2d(const cv::Point3f& point);


private:
    void publishDebugImage(const cv::Mat& image, const std_msgs::msg::Header& header);

    std::shared_ptr<Infer<yolo::BoxArray>>     yolo;
    std::shared_ptr<Infer<yolo::BoxArray>>     armor_yolo;
    std::shared_ptr<Infer<int>> classifier;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_img_pub_;

    bool        if_rosbag = false;
    int         if_foxglove = 0;
    int         EnemyColor;  // 0为蓝色 2为红色
    int         debug = 1;
    std::chrono::steady_clock::time_point last_debug_pub_time_{};
    std::string yolo_path;
    std::string armor_path;
    std::string classify_path;
};
class Car {
public:
    cv::Rect       car_rect;
    yolo::Box      car;
    yolo::BoxArray armors;
    cv::Point2f    center;
    cv::Rect       center_rect;
    int            number = 0;
    int            color = 1;
};
}  // namespace tdt_radar

#endif
