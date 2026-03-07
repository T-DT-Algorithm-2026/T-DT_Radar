#include "tdt_camera_node.h"
#include "TDT_CameraApi.h"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/header.hpp>
#include <filesystem>
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cstring>
#include <rclcpp_components/register_node_macro.hpp>

namespace tdt_vision {
using json = nlohmann::json;
using namespace std::chrono_literals;

TDTCameraNode::TDTCameraNode(const rclcpp::NodeOptions & options)
    : Node("tdt_camera_node", options) {
    
    this->declare_parameter<std::string>("config_path", "");
    this->declare_parameter<bool>("auto_start", false);

    std::string config_path;
    this->get_parameter("config_path", config_path);
    bool auto_start = this->get_parameter("auto_start").as_bool();

    // 如果 config_path 为空，尝试寻找默认路径
    if (config_path.empty()) {
        // 组件模式下，工作路径可能不确定，建议尽量通过 Launch 传参
        config_path = "config/config.json"; 
        RCLCPP_WARN(this->get_logger(), "未指定 config_path，尝试默认路径: %s", config_path.c_str());
        RCLCPP_WARN(this->get_logger(), "速速修改launch文件里的config路径: %s", config_path.c_str());
    }

    if (!std::filesystem::exists(config_path)) {
        RCLCPP_FATAL(this->get_logger(), "配置文件不存在: %s", config_path.c_str());
        // 组件构造函数抛出异常会导致容器加载失败，这是预期的
        throw std::runtime_error("Config file not found");
    }

    TDT_CAM::set_logger_mode(1);//谁不喜欢猫娘日志呢？ 
    RCLCPP_INFO(this->get_logger(), "正在初始化相机管理器...");
    
    manager_ = TDT_CAM::create_manager(config_path.c_str());
    if (!manager_) {
        RCLCPP_FATAL(this->get_logger(), "相机管理器创建失败！");
        throw std::runtime_error("Manager creation failed");
    }

    try {
        load_config(config_path);//加载相机配置
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "解析配置文件失败: %s", e.what());
        TDT_CAM::destroy_manager(manager_);
        throw;
    }

    //创建使用自定义接口的服务
    capture_srv_ = this->create_service<camera_interface::srv::CameraControl>(
        "set_capture",
        std::bind(&TDTCameraNode::set_capture_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "服务已就绪: set_capture (类型: camera_interface/CameraControl)");

    //这个服务用来重置帧号对齐
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_frame_offset", 
        std::bind(&TDTCameraNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "TDT_Camera 节点组件已初始化完成，等待启动指令...");

    if (auto_start) {
        RCLCPP_INFO(this->get_logger(), "检测到 auto_start=true，自动开始采集所有相机...");
        start_capture("all");
    }
}

TDTCameraNode::~TDTCameraNode() {
    // 析构时强制停止所有
    running_ = false; 
    if (manager_) {
        for (auto& cam : cameras_) {
            TDT_CAM::stop_grab(manager_, cam.id.c_str());
        }
    }
    for (auto& thread : capture_threads_) {
        if (thread.joinable()) thread.join();
    }
    if (manager_) {
        TDT_CAM::destroy_manager(manager_);
    }
}

//  服务回调实现
void TDTCameraNode::set_capture_callback(const std::shared_ptr<camera_interface::srv::CameraControl::Request> request,
                                         std::shared_ptr<camera_interface::srv::CameraControl::Response> response) {
    std::string target = request->camera_name;
    if (target.empty()) target = "all"; // 默认行为

    bool result = false;
    if (request->target_state) {
        result = start_capture(target);
        response->message = result ? "Start command executed for: " + target : "Failed to start (check logs)";
    } else {
        result = stop_capture(target);
        response->message = result ? "Stop command executed for: " + target : "Failed to stop (check logs)";
    }
    response->success = result;
}

// 启动逻辑 (支持单独启动)
bool TDTCameraNode::start_capture(const std::string& name) {
    if (cameras_.empty()) return false;

    // 确保线程已经跑起来了 (如果还没跑)
    if (!running_) {
        running_ = true;
        // 如果线程容器是空的，创建线程
        if (capture_threads_.empty()) {
            RCLCPP_INFO(this->get_logger(), "正在启动 %lu 个相机采集线程...", cameras_.size());
            for (size_t i = 0; i < cameras_.size(); ++i) {//为每个相机启动一个采集线程
                capture_threads_.emplace_back(
                    std::bind(&TDTCameraNode::capture_loop, this, i)//启动！
                );
            }
        }
    }

    int success_count = 0;
    int target_count = 0;

    for (auto& cam : cameras_) {
        // 匹配名字，"all" 匹配所有
        if (name == "all" || cam.id == name) {
            target_count++;
            if (!cam.is_capturing) {
                int ret = TDT_CAM::start_grab(manager_, cam.id.c_str());
                if (ret == TDT_CAM::OK) {
                    cam.is_capturing = true;
                    success_count++;
                    RCLCPP_INFO(this->get_logger(), "相机 [%s] 开始采集", cam.id.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "相机 [%s] 启动失败, ret=%d", cam.id.c_str(), ret);
                }
            } else {
                // 已经在跑了，也算成功
                success_count++;
            }
        }
    }

    if (target_count == 0) {
        RCLCPP_WARN(this->get_logger(), "未找到名为 '%s' 的相机", name.c_str());
        return false;
    }

    return success_count > 0;
}

// 停止逻辑 (支持单独停止)
bool TDTCameraNode::stop_capture(const std::string& name) {
    if (!running_) return true; // 根本没跑，视为成功

    int success_count = 0;
    int target_count = 0;

    for (auto& cam : cameras_) {
        if (name == "all" || cam.id == name) {
            target_count++;
            if (cam.is_capturing) {
                int ret = TDT_CAM::stop_grab(manager_, cam.id.c_str());
                // 无论返回值如何，我们在逻辑上都认为它停止了
                cam.is_capturing = false;
                success_count++;
                RCLCPP_INFO(this->get_logger(), "相机 [%s] 停止采集", cam.id.c_str());
            } else {
                success_count++;
            }
        }
    }

    if (target_count == 0) {
        RCLCPP_WARN(this->get_logger(), "未找到名为 '%s' 的相机", name.c_str());
        return false;
    }

    return true;
}

void TDTCameraNode::load_config(const std::string& config_path) {
    std::ifstream f(config_path);
    json data = json::parse(f);

    if (!data.contains("cameras") || !data["cameras"].is_array()) {
        RCLCPP_ERROR(this->get_logger(), "配置文件格式错误: 缺少 'cameras' 数组");
        return;
    }

    for (const auto& cam_config : data["cameras"]) {
        if (!cam_config.contains("id")) continue;

        std::string cam_id = cam_config["id"];
        CameraContext ctx;
        ctx.id = cam_id;
        ctx.frame_id = cam_id + "_optical_frame";
        ctx.topic_name = cam_id + "/image";
        
        // 虽然在 Publisher 端通常不是必须的（主要取决于 Subscription），但加上是个好习惯
        // 此处 create_publisher 不直接支持 intra_process 参数，它依赖 NodeOptions
        ctx.pub = this->create_publisher<sensor_msgs::msg::Image>(ctx.topic_name, 10);
        ctx.buffer.resize(20 * 1024 * 1024); 
        ctx.last_fps_time = std::chrono::steady_clock::now();
        // 初始化状态
        ctx.is_capturing = false; 

        cameras_.push_back(ctx);
        RCLCPP_INFO(this->get_logger(), "加载配置: [%s] (Topic: %s)", cam_id.c_str(), ctx.topic_name.c_str());
    }
}

void TDTCameraNode::capture_loop(size_t cam_idx) {
    auto& cam = cameras_[cam_idx]; 
    RCLCPP_INFO(this->get_logger(), "相机线程 [%s] 已启动", cam.id.c_str());

    while (rclcpp::ok() && running_) {
        if (!cam.is_capturing) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        int w, h, c, imu_id;
        uint64_t raw_id;
        double timestamp_sec;

        // 获取帧信息（不获取数据，很快）
        int ret = TDT_CAM::get_frame_info(manager_, cam.id.c_str(),
                                          &w, &h, &c, &raw_id, &imu_id, &timestamp_sec);

        bool has_new_frame = false;

        if (ret == TDT_CAM::OK) {
            if (raw_id != cam.last_frame_id) {
                size_t req_size = w * h * c;
                
                // 准备 Header
                std_msgs::msg::Header header;
                int64_t sec = static_cast<int64_t>(timestamp_sec);
                int64_t nanosec = static_cast<int64_t>((timestamp_sec - sec) * 1e9);
                header.stamp = rclcpp::Time(sec, nanosec);
                header.frame_id = cam.frame_id;
                std::string encoding = (c == 1) ? "mono8" : "bgr8";

                // 尝试 Zero Copy 发布
                if (cam.pub->can_loan_messages()) {
                    try {
                        auto loan_msg = cam.pub->borrow_loaned_message();
                        sensor_msgs::msg::Image& msg = loan_msg.get();
                        // 填充元数据
                        msg.header = header;
                        msg.height = h;
                        msg.width = w;
                        msg.encoding = encoding;
                        msg.is_bigendian = false;
                        msg.step = w * c;
                        msg.data.resize(req_size); // Loaned message resize通常只是调整逻辑大小

                        // 直接写入 Loaned 内存
                        if (TDT_CAM::get_frame_data(manager_, cam.id.c_str(), 
                                                    msg.data.data(), req_size) == TDT_CAM::OK) {
                            cam.pub->publish(std::move(loan_msg));
                            cam.last_frame_id = raw_id;
                            has_new_frame = true;
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Loan message failed: %s", e.what());
                    }
                }

                if (!has_new_frame) {
                    // 创建唯一指针消息，专为进程内通信优化
                    auto msg = std::make_unique<sensor_msgs::msg::Image>();
                    
                    msg->header = header;
                    msg->height = h;
                    msg->width = w;
                    msg->encoding = encoding;
                    msg->is_bigendian = false;
                    msg->step = w * c;
                    
                    // 直接调整消息内部 vector 的大小
                    msg->data.resize(req_size);

                    // 直接将 SDK 数据写入消息内存，不经过 cam.buffer 和 cv_bridge
                    if (TDT_CAM::get_frame_data(manager_, cam.id.c_str(), 
                                                msg->data.data(), req_size) == TDT_CAM::OK) {
                        
                        // 发布 unique_ptr，实现进程内零拷贝（指针传递）
                        cam.pub->publish(std::move(msg));
                        
                        cam.last_frame_id = raw_id;
                        has_new_frame = true;
                    }
                }
                
                // 统计 FPS
                if (has_new_frame) {
                    cam.fps_count++;
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - cam.last_fps_time).count();
                    
                    if (elapsed >= 1000) {
                        double fps = cam.fps_count * 1000.0 / elapsed;
                        RCLCPP_INFO(this->get_logger(), "[%s] 发布频率: %.2f Hz", cam.id.c_str(), fps);
                        cam.fps_count = 0;
                        cam.last_fps_time = now;
                    }
                }
            }
        }

        // 避免空转占用 CPU
        if (!has_new_frame) {
            // 1ms 的睡眠对于 165Hz (6ms/帧) 来说可能稍长，如果要求极高，可以由 yield 代替
            // std::this_thread::yield(); 
            std::this_thread::sleep_for(std::chrono::microseconds(500)); 
        }
    }
    RCLCPP_INFO(this->get_logger(), "相机线程 [%s] 已退出", cam.id.c_str());
}

void TDTCameraNode::reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_WARN(this->get_logger(), "收到硬同步重置请求 (所有相机)");
    // 重置需要硬重启所有正在运行的相机流
    for (auto& cam : cameras_) {
        if (cam.is_capturing) {
            TDT_CAM::stop_grab(manager_, cam.id.c_str());
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (auto& cam : cameras_) {
        if (cam.is_capturing) {
            TDT_CAM::start_grab(manager_, cam.id.c_str());
            RCLCPP_INFO(this->get_logger(), "相机 [%s] 已执行帧号对齐重置", cam.id.c_str());
        }
    }
    response->success = true;
    response->message = "Reset executed";
}

void TDTCameraNode::publish_image(CameraContext& cam, int w, int h, int c, double timestamp_sec, int imu_id) {
    std_msgs::msg::Header header;
    int64_t sec = static_cast<int64_t>(timestamp_sec);
    int64_t nanosec = static_cast<int64_t>((timestamp_sec - sec) * 1e9);
    header.stamp = rclcpp::Time(sec, nanosec);
    header.frame_id = cam.frame_id;
    std::string encoding = (c == 1) ? "mono8" : "bgr8";
    
    cv::Mat img;
    if (c == 1) img = cv::Mat(h, w, CV_8UC1, cam.buffer.data());
    else        img = cv::Mat(h, w, CV_8UC3, cam.buffer.data());

    try {
        sensor_msgs::msg::Image::SharedPtr msg = 
            cv_bridge::CvImage(header, encoding, img).toImageMsg();
        cam.pub->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}
}//namespace tdt_vision
RCLCPP_COMPONENTS_REGISTER_NODE(tdt_vision::TDTCameraNode)