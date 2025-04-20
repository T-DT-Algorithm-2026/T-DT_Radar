//基础相机类

#ifndef CAMERA_BASECAMERA_H_
#define CAMERA_BASECAMERA_H_

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
#include <string>
#include <thread>

// ROS
#include <cv_bridge/cv_bridge.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// interface
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tdt_vision {

struct Header {  //帧头信息
 public:
  void setHeader(uint64_t time_stamp, uint64_t image_seq) {
    this->time_stamp = time_stamp;
    this->image_seq = image_seq;
  }

 public:
  uint64_t time_stamp = 0;  // 时间戳
  uint64_t image_seq = 0;   // 帧序号
};

struct TImage {  //帧信息
 public:
  void setTImage(cv::Mat src, uint64_t step, uint64_t seq) {
    this->cvimage_ = src;
    this->header_.setHeader(step, seq);
  }

 public:
  Header header_;
  cv::Mat cvimage_;
};

/**
 * 相机类型: 0:usb相机 1:海康相机
 */
enum TDT_CAMERA_CAMTYPE {
  TDT_CAMERA_CAMTYPE_UVCCAM = 0,
  TDT_CAMERA_CAMTYPE_HIKVISION = 1
};

/**
 * 相机格式
 */
enum TDT_CAMERA_FORMAT {
  TDT_CAMERA_FORMAT_PIXEL = 0,
  TDT_CAMERA_FORMAT_WIDTH = 1,
  TDT_CAMERA_FORMAT_HEIGHT = 2,
  TDT_CAMERA_FORMAT_FPS = 3,
};

class Camera {
 public:
  // default constructor
  Camera() = default;

  // virtual void openCamera(std::string config_path) = 0;
  Camera(rclcpp::Node::SharedPtr predict_node);
  /*
  @brief: 获得相机画面
  格式：cv::Mat
  */
  virtual void GetImage(sensor_msgs::msg::Image& image_) = 0;

  inline double getTimeNow() {
    //单位：s
    return camera_node_->now().seconds();
  }

  void PubImage(const cv::Mat& image) {
    msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub->publish(*msg);
  }

  void PubImage(const sensor_msgs::msg::Image& msg) {
    sensor_msgs::msg::Image::UniquePtr msg_ =
        std::make_unique<sensor_msgs::msg::Image>(msg);
    // printf("pub1 pid: %d ,address: %p\n", getpid(), msg_.get());

    image_pub->publish(std::move(msg_));
    // printf("pub2 pid: %d ,address: %p\n", getpid(), msg_.get());
    // sensor_msgs::msg::Image::UniquePtr msg_ptr(msg);

    // std::stringstream ss;
    // // Put this process's id and the msg's pointer address on the image.
    // ss << "pid: " << getpid() << ", ptr: " << msg_ptr.get();

    // image_pub->publish(std::move(msg_ptr));
  }
  /*
  @brief: 发布相机信息
  */
  virtual void publishCameraInfo() = 0;
  /*
  @brief：打开相机
  */
  virtual bool OpenCamera() = 0;

  /*
  @brief: 关闭相机
  */
  virtual bool CloseCamera() = 0;
  rclcpp::Node::SharedPtr camera_node_ = NULL;

 protected:
  struct CameraIdentity {  //相机身份信息
    TDT_CAMERA_CAMTYPE type;
    std::string name;
    std::string guid;
    std::string config_name;
  } identity_;

  struct CameraFormat {  //相机格式信息
    bool pixel_format;
    int width;
    int height;
    float fps;
  } format_;

  bool Stop = false;
  std::mutex stop_mtx_;
  std::mutex img_mtx_;
  std::condition_variable condVar_;
  cv::Mat takeImg_, swapImg_, srcImg_;
  double takeTime_, swapTime_, srcTime_;
  bool first_run_ = true;
  double triggerTime_;

 protected:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  sensor_msgs::msg::Image::SharedPtr msg;
};

}  // namespace tdt_vision

#endif