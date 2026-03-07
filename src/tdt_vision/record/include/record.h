#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <fstream>
#include "gimbal_interface/msg/gimbal_angle.hpp"

namespace tdt_radar{

const cv::Size BOARD_SIZE = cv::Size(11, 8);     // 内部角点数量
const float SQUARE_SIZE = 0.025f;                  // 标定板方格边长 (单位：米)
float yaw;
float pitch;
float last_yaw;
float last_pitch;
int number=0;

class Record : public rclcpp::Node {
public:
    // 构造函数
    explicit Record(const rclcpp::NodeOptions& options);
private:
    // 回调函数声明
    void callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);

    // 订阅者成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_sub;

};
}// namespace tdt_radar