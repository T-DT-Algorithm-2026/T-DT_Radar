#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "gimbal_interface/msg/gimbal_angle.hpp"
#include "vision_interface/msg/resolve_result.hpp"
#include "vision_interface/msg/detect_fly.hpp"
#include "kalman_cv.h"

#include "rclcpp_components/register_node_macro.hpp"

namespace tdt_lock{

struct Gimbal 
{
    float yaw;
    float pitch;
    double time;
};

class Lock : public rclcpp::Node {
public:
    explicit Lock(const rclcpp::NodeOptions& options);
private:
    rclcpp::Subscription<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_sub;
    rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_pub;
    rclcpp::Subscription<vision_interface::msg::DetectFly>::SharedPtr fly_sub;

    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);
    void callback(const vision_interface::msg::DetectFly::SharedPtr msg);
    Gimbal find_closest_time(double target_time);

    std::mutex gimbal_mutex;
    std::deque<Gimbal> gimbal_history;

    int is_find = 0;
    float base_yaw = 0;
    float base_pitch = 0;
    bool is_first = false;
    std::shared_ptr<Kalman_filter_plus> kf_ptr = nullptr;

    float kp_x = 1.6;
    float kp_y = 1.6;
    float kd = 0.03;
    float fx;
    float fy;
    float target_x;
    float target_y;
    float cx = 720.0f;
    float cy = 540.0f;//参数
    float last_dyaw = 0;
    float last_dpitch = 0;
};

}// namespace tdt_lock