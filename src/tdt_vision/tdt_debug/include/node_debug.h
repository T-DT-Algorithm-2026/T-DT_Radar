#ifndef VISION_DEBUG_H
#define VISION_DEBUG_H

// roborts_utils
#include "roborts_utils/roborts_utils.h"

// hik
#include <MvCameraControl.h>

// opencv
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <stdlib.h>

#include <cinttypes>
#include <condition_variable>
#include <deque>
#include <string>
#include <thread>

// ROS
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tdt_vision {

class NodeDebug : public rclcpp::Node {
 public:
  NodeDebug(const rclcpp::NodeOptions &node_options);

  ~NodeDebug();

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void image2_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void image_pub(const sensor_msgs::msg::Image::SharedPtr msg);

  void image2_pub(const sensor_msgs::msg::Image::SharedPtr msg);

  void record();

 private:  // topic sub & pub
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image2_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image2_pub_;

  bool image_update_ = false;
  bool image2_update_ = false;
  std::deque<sensor_msgs::msg::Image> image_buffer_;
  std::deque<sensor_msgs::msg::Image> image2_buffer_;
  const int image_buffer_size = 10;

  std::shared_ptr<std::thread> debug_thread_;
};



}  // namespace tdt_vision

#endif
