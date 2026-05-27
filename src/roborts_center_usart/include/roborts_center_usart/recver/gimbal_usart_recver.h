#ifndef __GIMBAL_USART_RECVER_H
#define __GIMBAL_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "roborts_center_usart/base_usart.h"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "gimbal_interface/msg/gimbal_angle.hpp"

namespace tdtusart {
class GimbalUsartRecver : public BaseUsartRecver {
 public:
  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    gimbalUsartData = node->create_publisher<gimbal_interface::msg::GimbalAngle>(
        "gimbalUsartData", 1);
  }
#pragma pack(1)
  struct GimbalData {
    uint8_t header = 0xA5;
    uint8_t type = 3;

    uint8_t force_flag;

    float yaw;
    float pitch;
    bool is_fire;

    float time_stamp;
    uint32_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(GimbalData); }

  void ParseData(void *message) override {
    GimbalData *gimbal_data = (GimbalData *)message;
    gimbal_interface::msg::GimbalAngle usrtPub;
    usrtPub.yaw = gimbal_data->yaw;
    usrtPub.pitch = -gimbal_data->pitch;
    usrtPub.is_fire = gimbal_data->is_fire;
    usrtPub.force_flag = gimbal_data->force_flag;
    usrtPub.header.stamp = tdttoolkit::Time::GetRosTimeByTime(gimbal_data->time_stamp*1e6);
    // std::cout<<"cameraID:"<<gimbal_data->cameraID<<"signal:"<<gimbal_data->recieve_request_signal;
    // std::cout<<"Recive Gimbal Data: yaw="<<usrtPub.yaw<<" pitch="<<usrtPub.pitch<<std::endl;
    // std::cout<<"time:"<<usrtPub.header.stamp.sec << "." << usrtPub.header.stamp.nanosec<<std::endl;

    // 1. 构造时钟对象（建议使用 ROS 时钟，以便兼容仿真和回放）
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    // 2. 获取当前时刻
    rclcpp::Time now = ros_clock.now();

    // 3. 按照你要求的格式输出 (与 usrtPub.header.stamp 结构完全一致)
    // std::cout << "Seconds: " << (uint32_t)now.seconds() << "." << (now.nanoseconds() % 1000000000) << std::endl;

    gimbalUsartData->publish(usrtPub);
    TDT_INFO("Gimbal Received&&Pub!");
  }
  
  rclcpp::Publisher<gimbal_interface::msg::GimbalAngle>::SharedPtr gimbalUsartData;
  };
}  // namespace tdtusart

#endif
