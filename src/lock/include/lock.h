#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "gimbal_interface/msg/gimbal_angle.hpp"
#include "vision_interface/msg/detect_fly.hpp"
#include "vision_interface/msg/match_info.hpp"
#include "kalman_cv.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
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
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_info_sub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;
    rclcpp::Time last_lidar_time_;
    rclcpp::Time last_match_info_time_;

    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);
    void callback(const vision_interface::msg::DetectFly::SharedPtr msg);
    void timer_callback();
    void find_callback();
    void lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg);
    void match_info_callback(const vision_interface::msg::MatchInfo::SharedPtr msg);
    Gimbal find_closest_time(double target_time);
    void patrol();
    void read_config();

    std::mutex gimbal_mutex;
    std::deque<Gimbal> gimbal_history;

    std::shared_ptr<Kalman_filter_plus> kf_ptr = nullptr;

    // 云台角度误差的比例增益。增大后追踪更快，但过大会产生过冲和往复抖动。
    double kp_x = 1.0;
    double kp_y = 1.0;

    // 总预测时间 = 图像时间戳到当前时刻的延迟 + control_delay_s。
    // 限制最大值可以避免消息严重积压或时间戳异常时预测过远。
    double max_prediction_horizon_s = 0.15;

    // 控制角度死区。目标误差小于此值时不继续追逐视觉检测的微小跳动。
    double angle_deadband_rad = 0.0 * CV_PI / 180.0;

    //卡尔曼参数
    double kf_measurement_noise_px;
    double kf_measurement_noise_y_px;
    double kf_q_x_rad2_s3;
    double kf_q_y_rad2_s3;
    double kf_initial_velocity_std_deg_s;
    double control_delay_s = 0.015;
    double countermeasure_interval_s = 10.0;
    double match_info_timeout_s = 2.0;
    double first_lock_yaw_offset_deg = -2.0;
    double first_lock_pitch_offset_deg = 0.2;

    bool enemy_drone_countered = false;
    bool countermeasure_waiting = false;
    bool sentry_go_kill_received = false;
    bool is_fire = true;
    int countermeasure_count = 0;
    rclcpp::Time countermeasure_end_time;

    // read_config() 完成单位转换后，将上述参数汇总到卡尔曼实际使用的配置中。
    AngleKalmanConfig kf_config;
    float fx;
    float fy;
    
    float target_x=1053;
    float target_y=500;
    float A_x = 0.0f, B_x = 1053.0f;
    float A_y = 0.0f, B_y = 500.0f;//激光落点计算

    float cx = 720.0f;
    float cy = 540.0f;//参数
    float yaw_cmd_first = 0;
    float pitch_cmd_first = 0;
    int patrol_state_ = 0;

    // cv::Point3f fly_pos = cv::Point3f(15.0f, -4.0f, -1.0f);
    cv::Point3f fly_pos = cv::Point3f(0.0f, 0.0f, 0.0f);
    bool lidar_valid = false;
    bool radar_angle_valid = false;
    float radar_yaw = 0.0f;
    float radar_pitch = 0.0f;
    // float radar_yaw_limit = 4.0f;
    // float radar_pitch_limit = 4.0f;

    // TF related
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters for non-coaxial gimbal
    cv::Mat cam2pitch_rvec_ = cv::Mat::zeros(3, 1, CV_64F); // 默认无旋转
    geometry_msgs::msg::TransformStamped static_transform_;
    void publish_static_tf();
};

}// namespace tdt_lock
