#include "baseCamera.h"
namespace tdt_vision {

Camera::Camera(rclcpp::Node::SharedPtr camera_node)
    : camera_node_(camera_node) {
  image_pub = camera_node_->create_publisher<sensor_msgs::msg::Image>(
      "camera_image", rclcpp::SensorDataQoS());
  tdttoolkit::Time::Init(tdttoolkit::Time::GetROSTimeNow());
}

}  // namespace tdt_vision