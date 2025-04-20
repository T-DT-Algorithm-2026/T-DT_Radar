#ifndef VISION_CAMERA_H
#define VISION_CAMERA_H

// ROS2
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// STL
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

// camera
#include "baseCamera.h"
#include "hikCamera.h"
#include "videoCamera.h"

// roborts_utils
#include <roborts_utils/base_msg.h>
#include <roborts_utils/base_param.h>
#include <roborts_utils/base_toolkit.h>
#include <roborts_utils/config.h>

namespace tdt_vision {

class NodeCamera final : public rclcpp::Node {
 public:
  explicit NodeCamera(const rclcpp::NodeOptions& options);

  ~NodeCamera();

 private:
  void param_init();

  void camera_node_init();

  void getImage();

  void timeMatch();
  
  void pubImage();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  
  // thread
  std::shared_ptr<std::thread> camera_thread_;  //相机取帧线程

  // ROS2
  std::shared_ptr<HikvisionCam> HikvisionCam_;
  std::shared_ptr<VideoCamera> VideoCamera_;
  std::shared_ptr<Camera> Camera_;

  // info
  std::string param_name_ = "Camera";

  std::string param_path_ = "./config/camera_param.jsonc";


  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rclcpp::TimerBase::SharedPtr button_timer_;


};
}  // namespace tdt_vision

#endif
