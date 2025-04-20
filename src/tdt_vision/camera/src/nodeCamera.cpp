// Headers
#include "nodeCamera.h"

#include <rclcpp/qos.hpp>

namespace tdt_vision {

NodeCamera::NodeCamera(const rclcpp::NodeOptions &node_options)
    : Node("vision_camera_node", node_options) {
      
  camera_thread_ = std::make_shared<std::thread>(&NodeCamera::getImage, this);
}

NodeCamera::~NodeCamera() { camera_thread_->join(); }

void NodeCamera::getImage() {
  std::chrono::milliseconds time(100);
  // 延迟100ms,为了完整地执行完构造函数，可以安全地使用
  // shared_from_this() 来获取指向当前对象的智能指针
  std::this_thread::sleep_for(time);
  RCLCPP_INFO(this->get_logger(), "camera_node init");

  param_init();
  camera_node_init();

  rclcpp::WallRate loop_rate(200);
  // 循环频率为200Hz
  // 选择相机模式
  if (tdtconfig::CAMERA)
    Camera_ = HikvisionCam_;
  else
    Camera_ = VideoCamera_;

  Camera_->OpenCamera();

  while (rclcpp::ok()) {
    sensor_msgs::msg::Image image_;
    auto start1 = std::chrono::system_clock::now();

    Camera_->GetImage(image_);
    // cv::Mat 格式
    if (image_.width == 0 || image_.height == 0 || image_.data.size() == 0) {
      std::cout << "image is empty" << std::endl;
      continue;
    }

    auto start = std::chrono::system_clock::now();

    Camera_->PubImage(image_);
    Camera_->publishCameraInfo();


    auto end = std::chrono::system_clock::now();

    // loop_rate.sleep();
  }
  Camera_->CloseCamera();
  rclcpp::shutdown();
}

void NodeCamera::param_init() {
  if(!tdtconfig::ifInit)
    tdtconfig::Init();
  LoadParam::InitParam(param_name_, param_path_);
  LoadParam::InitROSParam(param_name_, shared_from_this());
  this->declare_parameter<int>("SaveParamNow", 0);
  button_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this] { this->set_parameter(rclcpp::Parameter("SaveParamNow", 0)); });
  callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &NodeCamera::parametersCallback, this, std::placeholders::_1));
}

void NodeCamera::camera_node_init() {
  if (tdtconfig::CAMERA) {
    HikvisionCam_ =
        std::make_shared<HikvisionCam>(param_name_, shared_from_this());
  } else {
    VideoCamera_ = std::make_shared<VideoCamera>(param_name_, shared_from_this());
  }
}



rcl_interfaces::msg::SetParametersResult NodeCamera::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  if (this->get_parameter("SaveParamNow").get_value<int>()) {
    LoadParam::OutPutROSParam(param_name_, shared_from_this());
  }
  return result;
}

}  // namespace tdt_vision

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_vision::NodeCamera)
