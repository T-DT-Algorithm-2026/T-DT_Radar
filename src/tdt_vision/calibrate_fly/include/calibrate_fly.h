#ifndef RADAR_CALIBRATE_FLY_H
#define RADAR_CALIBRATE_FLY_H

#include <memory>
#include <string>
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tdt_radar {
    // Global or static variables used for the mouse callback
    static cv::Mat cvimage_fly_;
    static bool is_calibrating_fly = false;
    static cv::Point2f pick_point_fly(-1, -1);
    static bool point_selected_fly = false;

    class CalibrateFly final : public rclcpp::Node {
    public:
        explicit CalibrateFly(const rclcpp::NodeOptions &options);

        void callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;

        void solve();
    };

    void fly_mousecallback(int event, int x, int y, int flags, void *userdata);

}  // namespace tdt_radar
#endif
