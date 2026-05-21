#ifndef RADAR_CALIBRATE_H
#define RADAR_CALIBRATE_H

#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/photo.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "radar_utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace tdt_radar {
    static cv::Mat cvimage_;
    static std::vector<cv::Point2f> pick_points;
    static cv::Mat camera_matrix;
    static cv::Mat dist_coeffs;
    static cv::Mat rvec;
    static cv::Mat tvec;
    static bool is_calibrating = false;
    static bool is_location = false;
    static std::vector<cv::Point2f> locate_points;


    class Calibrate final : public rclcpp::Node {
    public:
        std::vector<cv::Point3f> real_points;
        

        // cv::Point3f self_FORTRESS   = cv::Point3f(5.471, -7.5, 0.0);
        // cv::Point3f self_Tower = cv::Point3f(10.936, -11.161, 0.868);
        // cv::Point3f enemy_Base  = cv::Point3f(25.49, -7.5, 1.24524);
        // cv::Point3f enemy_Tower =cv::Point3f(16.925, -3.625, 1.745);
        // cv::Point3f enemy_High = cv::Point3f(20.20, -10.8, 0.8);
        cv::Point3f right_low = cv::Point3f(7.23, -11.635, 0.18);
        cv::Point3f midle= cv::Point3f(10.206, -5.92, 0.338);
        cv::Point3f buffer = cv::Point3f(13.296, -8, 2.246);
        cv::Point3f right_behind = cv::Point3f(24.6, -4, 0.57);
        cv::Point3f left_behind = cv::Point3f(22.7, -12.52,0.436);

        
        explicit Calibrate(const rclcpp::NodeOptions &options);

        void callback(const sensor_msgs::msg::Image::SharedPtr msg);

        void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;

        void solve();

        parser *parser_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

        geometry_msgs::msg::TransformStamped transformStamped;

        void store_locate();

        void draw_locate(cv::Mat& img);

    };
    void mousecallback(int event, int x, int y, int flags, void *userdata);

}  // namespace tdt_radar
#endif
