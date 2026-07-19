#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <mutex>
#include <algorithm>
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;

    void gimbal_callback(const gimbal_interface::msg::GimbalAngle::SharedPtr msg);
    void callback(const vision_interface::msg::DetectFly::SharedPtr msg);
    void timer_callback();
    void find_callback();
    void lidar_callback(const geometry_msgs::msg::Point32::SharedPtr msg);
    Gimbal find_closest_time(double target_time);
    void patrol();

    std::mutex gimbal_mutex;
    std::deque<Gimbal> gimbal_history;

    std::shared_ptr<Kalman_filter_plus> kf_ptr = nullptr;

    // 云台角度误差的比例增益。增大后追踪更快，但过大会产生过冲和往复抖动。
    double kp_x = 1.0;
    double kp_y = 1.0;

    // 图像处理完成后，串口传输和云台机械响应仍需时间，因此额外向未来预测 10 ms。
    // 目标仍落后时可适当增大；出现明显超前或越过目标时应减小。
    double control_delay_s = 0.01;

    // 总预测时间 = 图像时间戳到当前时刻的延迟 + control_delay_s。
    // 限制最大值可以避免消息严重积压或时间戳异常时预测过远。
    double max_prediction_horizon_s = 0.15;

    // 控制角度死区。目标误差小于此值时不继续追逐视觉检测的微小跳动。
    double angle_deadband_rad = 0.04 * CV_PI / 180.0;

    // 视觉中心点的单轴标准差，单位为像素。增大后更相信运动模型、输出更平滑，
    // 但对目标真实转向的响应会变慢；构造函数中会通过 fx/fy 换算成角度标准差。
    double kf_measurement_noise_px = 4.0;

    // 连续白噪声角加速度强度，单位 rad^2/s^3，对应卡尔曼 Q 矩阵。
    // 增大后允许角速度更快变化，转向跟随更及时，但速度估计和输出也更容易抖动。
    double kf_angular_acceleration_noise = 0.05;

    // 卡尔曼刚建立或重置时的角速度标准差。10 deg/s 可以覆盖当前目标理论最大角速度。
    double kf_initial_velocity_std_rad_s = 10.0 * CV_PI / 180.0;

    // 测量创新门限 = 基础门限 + 角速度门限 * dt。
    // 超出门限的单帧结果视为误检；连续三帧超限才用新位置重置滤波器。
    double kf_innovation_gate_base_rad = 0.15 * CV_PI / 180.0;
    double kf_innovation_gate_rate_rad_s = 10.0 * CV_PI / 180.0;

    // 汇总后的卡尔曼配置，节点初始化时由上面的固定参数赋值。
    AngleKalmanConfig kf_config;
    float fx;
    float fy;
    
    float target_x=854;
    float target_y=502;
    float dist1, target_x1, target_y1;
    float dist2, target_x2, target_y2;
    float A_x = 0, B_x = 0;
    float A_y = 0, B_y = 0;//激光落点计算

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
