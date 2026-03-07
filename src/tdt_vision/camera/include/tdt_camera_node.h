#ifndef TDT_CAMERA_NODE_H
#define TDT_CAMERA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
// 1. 引入自定义接口头文件
#include "camera_interface/srv/camera_control.hpp"
#include "TDT_CameraApi.h"

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
namespace tdt_vision {
struct CameraContext {
    std::string id;
    std::string frame_id;
    std::string topic_name;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    std::vector<uint8_t> buffer;
    uint64_t last_frame_id = 0;
    int fps_count = 0;
    std::chrono::steady_clock::time_point last_fps_time;
    // 2. 新增：单个相机的运行状态标志
    bool is_capturing = false; 
};

class TDTCameraNode : public rclcpp::Node {
public:
    explicit TDTCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~TDTCameraNode();

private:
    void load_config(const std::string& config_path);
    void capture_loop(size_t cam_idx);
    void publish_image(CameraContext& cam, int w, int h, int c, double timestamp_sec, int imu_id);
    
    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // 使用自定义服务的回调
    void set_capture_callback(const std::shared_ptr<camera_interface::srv::CameraControl::Request> request,
                              std::shared_ptr<camera_interface::srv::CameraControl::Response> response);
    
    //支持指定相机名称的控制函数
    bool start_capture(const std::string& name = "all");
    bool stop_capture(const std::string& name = "all");

    void* manager_ = nullptr;
    std::vector<CameraContext> cameras_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    //服务对象类型
    rclcpp::Service<camera_interface::srv::CameraControl>::SharedPtr capture_srv_;

    std::atomic<bool> running_{false}; // 控制线程是否存在
    std::vector<std::thread> capture_threads_;
};
}//namespace tdt_vision
#endif // TDT_CAMERA_NODE_H