#include "videoCamera.h"

#include <opencv2/highgui.hpp>

namespace tdt_vision {

VideoCamera::VideoCamera(std::string param_name, rclcpp::Node::SharedPtr camera_node)
    : Camera(camera_node) {
  LoadParam::ReadParam(param_name, "VideoPath", video_path_);
  RCLCPP_INFO(camera_node_->get_logger(), "videoCamera_node init");
}

bool VideoCamera::OpenCamera() {
  video_.open(video_path_);
  return video_.isOpened();
}

bool VideoCamera::CloseCamera() {
  if (video_.isOpened()) {
    video_.release();
    return true;
  } else {
    return false;
  }
}

void VideoCamera::GetImage(sensor_msgs::msg::Image &image_) {
  if (video_.isOpened()) {
    static cv::Mat last_frame;
    cv::Mat frame;
    video_ >> frame;
    if (!frame.empty()) {
      last_frame = frame;
    }else{
      video_.set(cv::CAP_PROP_POS_FRAMES, 0);//循环播放
    }
    if (!last_frame.empty()) {
      image_ = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", last_frame)
                     .toImageMsg());
      image_.header.stamp = camera_node_->now();
    }
    // cv::imshow("",frame);
    // cv::waitKey(0);
  }
}

}  // namespace tdt_vision