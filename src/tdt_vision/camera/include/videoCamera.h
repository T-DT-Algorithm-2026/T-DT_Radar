#ifndef VIDEO_CAMERA_H
#define VIDEO_CAMERA_H

#include <opencv2/opencv.hpp>

#include "baseCamera.h"

namespace tdt_vision {

class VideoCamera : public Camera {
 public:
  VideoCamera(std::string param_name, rclcpp::Node::SharedPtr camera_node);

  /*
  @brief: 打开视频
  */
  bool OpenCamera() override;

  /*
  @brief：关闭视频
  */
  bool CloseCamera() override;

  /*
  @brief:获取视频帧
  */
  void GetImage(sensor_msgs::msg::Image& image_) override;

  void publishCameraInfo() override {};
  
 private:
  std::string video_path_;
  cv::VideoCapture video_;
  uint64_t image_seq_ = 0;
};

}  // namespace tdt_vision

#endif
