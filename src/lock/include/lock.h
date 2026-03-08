#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "gimbal_interface/msg/gimbal_angle.hpp"
#include "vision_interface/msg/resolve_result.hpp"
#include "kalman_fly.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h> 
#include <tf2_ros/buffer.h>            
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "rclcpp_components/register_node_macro.hpp"

namespace tdt_lock{

class Lock : public rclcpp::Node {
public:
    explicit Lock(const rclcpp::NodeOptions& options);
private:
    rclcpp::Subscription<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_sub;
    rclcpp::Subscription<vision_interface::msg::ResolveResult>::SharedPtr resolve_sub;
    rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbal_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void callback(const vision_interface::msg::ResolveResult::SharedPtr msg);
    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);
    void load_camera_params(); 
    void publish_static_tf();
    void timer_callback();

    cv::Mat cam2pitch_rvec_;
    cv::Mat cam2pitch_tvec_;
    cv::Mat pitch2yaw_tvec_ = (cv::Mat_<double>(3,1) << 0.0, -42.9, 151.75); // 假设 Pitch 到 Yaw 的平移为零
    // float yaw=0;
    float yaw;
    float pitch;
    std::shared_ptr<Kalman_filter_plus> kf_ptr = nullptr;

};

}// namespace tdt_lock