#include "node_debug.h"

#include <roborts_utils/base_msg.h>

#include <numeric>
#include <rclcpp/qos.hpp>

namespace tdt_vision {

NodeDebug::NodeDebug(const rclcpp::NodeOptions &node_options)
    : Node("debug_node", node_options) {
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera1/image", rclcpp::SensorDataQoS(),
      std::bind(&NodeDebug::image_callback, this, std::placeholders::_1));
  image2_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera2/image", rclcpp::SensorDataQoS(),
      std::bind(&NodeDebug::image2_callback, this, std::placeholders::_1));

  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "compressed_image", 10);
  image2_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "compressed_image2", 10);

  debug_thread_ = std::make_shared<std::thread>(&NodeDebug::record, this);
}

NodeDebug::~NodeDebug() { debug_thread_->join(); }

void NodeDebug::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  image_buffer_.push_back(*msg);

  if (image_buffer_.size() > image_buffer_size) {
    image_buffer_.pop_front();
  }
  image_update_ = true;
}

void NodeDebug::image2_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  image2_buffer_.push_back(*msg);

  if (image2_buffer_.size() > image_buffer_size) {
    image2_buffer_.pop_front();
  }
  image2_update_ = true;
}

void NodeDebug::record() {
  std::chrono::milliseconds time(100);
  std::this_thread::sleep_for(time);
  RCLCPP_INFO(this->get_logger(), "debug_node init!");

  rclcpp::WallRate loop_rate(200);

  while (rclcpp::ok()) {
    if (image_update_) {
      this->image_pub(
          std::make_shared<sensor_msgs::msg::Image>(image_buffer_.back()));
    }
    if (image2_update_) {
      this->image2_pub(
          std::make_shared<sensor_msgs::msg::Image>(image2_buffer_.back()));
    }
    // loop_rate.sleep();
  }
}

void NodeDebug::image_pub(const sensor_msgs::msg::Image::SharedPtr msg) {
  sensor_msgs::msg::CompressedImage compressed_image;
  compressed_image.header = msg->header;
  compressed_image.format = "jpeg";
  cv::Mat image(msg->height, msg->width, CV_8UC3, msg->data.data());
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,65}; 
  std::vector<uchar> buf;
  cv::imencode(".jpeg", image, buf, params);
  compressed_image.data.assign(buf.begin(), buf.end());

  image_pub_->publish(compressed_image);
  TDT_INFO("Compressed Image Publish!");
  image_update_ = false;
}

void NodeDebug::image2_pub(const sensor_msgs::msg::Image::SharedPtr msg) {
  sensor_msgs::msg::CompressedImage compressed_image;
  compressed_image.header = msg->header;
  compressed_image.format = "jpeg";
  cv::Mat image(msg->height, msg->width, CV_8UC3, msg->data.data());
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY,65};
  std::vector<uchar> buf;
  cv::imencode(".jpeg", image, buf, params);
  compressed_image.data.assign(buf.begin(), buf.end());

  image2_pub_->publish(compressed_image);
  // TDT_INFO("Compressed Image2 Publish!");
  image2_update_ = false;
}

}  // namespace tdt_vision

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_vision::NodeDebug)
