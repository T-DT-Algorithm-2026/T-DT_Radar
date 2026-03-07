// #ifndef __GIMBAL_USART_SENDER_H
// #define __GIMBAL_USART_SENDER_H

// #include <roborts_utils/base_msg.h>

// #include <boost/asio.hpp>
// #include <rclcpp/qos.hpp>

// #include "base_usart.h"
// #include "crc_tools.h"
// #include "roborts_utils/roborts_utils.h"
// #include "usart.h"
// #include "gimbal_interface/msg/gimbal_angle.hpp"

// // sender 向串口发送消息
// namespace tdtusart {
// class GimbalUsartSender : public BaseUsartSender {
//  public:
//   void init_communicator(
//       std::shared_ptr<rclcpp::Node> &node,
//       std::function<bool(const void *, int)> usartSend) override {
//     subscriber_ =
//         node->create_subscription<gimbal_interface::msg::GimbalAngle>(
//             "GimbalPub",       // topic_name,
//             rclcpp::SensorDataQoS(),  // buff_size,
//             std::bind(&GimbalUsartSender::Callback, this,
//                       std::placeholders::_1));
//     usartSend_ = usartSend;
//   }
// #pragma pack(1)
//   struct GimbalData {
//     uint8_t header = 0xA5;
//     uint8_t type = 4;

//     float yaw;
//     float pitch;
//     bool is_fire;

//     float time_stamp;
//     uint32_t frame_id;
//     uint16_t CRC16CheckSum;
//   };
// #pragma pack()

//  private:
//   // tdttoolkit::BaseCommunicator *communicator;
//   rclcpp::Subscription<gimbal_interface::msg::GimbalAngle>::SharedPtr
//       subscriber_;

//   std::function<bool(const void *, int)> usartSend_;


//   void Callback(const std::shared_ptr<const gimbal_interface::msg::GimbalAngle> msg) {
//     GimbalData send_data;
//     send_data.yaw = msg->yaw;
//     send_data.pitch = -msg->pitch;
//     send_data.is_fire = msg->is_fire;
//     // send_data.yaw = 0;
//     // send_data.pitch = 0;
//     // send_data.is_fire = 1;
//     send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
//     CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
//     //std::cout<<sizeof(send_data)<<"  "<<send_data.joints_rad[0]<<std::endl;
//     std::cout<<"Gimbal Send yaw:"<<send_data.yaw<<" pitch:"<<send_data.pitch<<std::endl;
//     std::cout<<"send"<<msg->yaw<<" "<<msg->pitch<<std::endl;


//     usartSend_(&send_data, sizeof(send_data));
//     TDT_INFO("Send Gimbal Data");
//   }
// };

// }  // namespace tdtusart
// #endif

#ifndef __GIMBAL_USART_SENDER_H
#define __GIMBAL_USART_SENDER_H

#include <roborts_utils/base_msg.h>

#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> // 新增：用于定时器的时间单位

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "gimbal_interface/msg/gimbal_angle.hpp"

namespace tdtusart {
class GimbalUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    
    // 保存发送函数句柄
    usartSend_ = usartSend;

    // 关键修改：创建一个定时器，主动、周期性地触发发送函数。
    // 这里设置为 10 毫秒 (100Hz) 发送一次，你可以根据电控需求修改这个频率
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&GimbalUsartSender::TimerCallback, this));
  }

#pragma pack(1)
  struct GimbalData {
    uint8_t header = 0xA5;
    uint8_t type = 4;

    float yaw;
    float pitch;
    bool is_fire;

    float time_stamp;
    uint32_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // 将原来的 subscriber_ 替换为 timer_
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;

  // 新增：定时器回调函数，不需要传入 msg 参数
  void TimerCallback() {
    GimbalData send_data;
    
    // 强制写入你需要的死值
    send_data.yaw = 0.0f;
    send_data.pitch = -1.0f;
    send_data.is_fire = 1;
    
    send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    
    // 打印调试信息
    std::cout << "Gimbal Auto Send -> yaw: " << send_data.yaw 
              << ", pitch: " << send_data.pitch 
              << ", is_fire: " << send_data.is_fire << "\r" << std::flush;

    // 执行串口发送
    if (usartSend_) {
        usartSend_(&send_data, sizeof(send_data));
    }
  }
};

}  // namespace tdtusart
#endif