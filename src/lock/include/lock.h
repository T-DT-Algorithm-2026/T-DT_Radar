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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

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
    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr lidar_sub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;

    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);
    void callback(const vision_interface::msg::DetectFly::SharedPtr msg);
    void timer_callback();
    void find_callback();
    void lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg);
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

    // cv::Point3f object_fly = cv::Point3f(4.0f, 15.0f, -1.0f);
    cv::Point3f fly_pos;

    // TF related
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters for non-coaxial gimbal
    cv::Mat cam2pitch_rvec_ = cv::Mat::zeros(3, 1, CV_64F); // 默认无旋转
    void publish_static_tf();
};

}// namespace tdt_lock