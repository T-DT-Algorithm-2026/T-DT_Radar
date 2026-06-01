#ifndef __RADIO_BUFF_SENDER_H
#define __RADIO_BUFF_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/buff.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioBuffSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    node_ = node;  // 新增

    subscriber_ = node->create_subscription<radio_interface::msg::Buff>(
        "radio_buff",
        rclcpp::SensorDataQoS(),
        std::bind(&RadioBuffSender::Callback, this, std::placeholders::_1));

    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RadioBuffSender::TimerCallback, this));

    usartSend_ = usartSend;
    last_msg_time_ = node_->now();  // 新增
  }

#pragma pack(1)

  struct RadioBuff {
    uint8_t header = 0xA5;
    uint8_t type = 1;

    uint8_t individe[5];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  std::shared_ptr<rclcpp::Node> node_;  // 新增

  rclcpp::Subscription<radio_interface::msg::Buff>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  uint8_t individe_state_[5] = {0, 0, 0, 0, 0};  // 新增
  rclcpp::Time last_msg_time_;                   // 新增

  void Callback(const std::shared_ptr<const radio_interface::msg::Buff> msg) {
    last_msg_time_ = node_->now();  // 新增：记录收到消息的时间

    RadioBuff send_data;

    for (int i = 0; i < 5; i++) {
      if (msg->defence[i] == 100) {
        send_data.individe[i] = 1;
        individe_state_[i] = 1;  // 新增：保存状态
      } else {
        send_data.individe[i] = 0;
        individe_state_[i] = 0;  // 新增：保存状态
      }
    }

    send_data.frame_id = frame_id_++;

    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));

    if (usartSend_) {
      usartSend_(&send_data, sizeof(send_data));
    }
  }

  void TimerCallback()
  {
    if (!node_) {
      return;
    }

    RadioBuff send_data;

    // 如果 0.3 秒没收到 radio_buff，就全部判断为非无敌车
    if ((node_->now() - last_msg_time_).seconds() > 0.3) {
      for (int i = 0; i < 5; i++) {
        individe_state_[i] = 0;
      }
    }

    for (int i = 0; i < 5; i++) {
      send_data.individe[i] = individe_state_[i];
    }

    send_data.frame_id = frame_id_++;

    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));

    if (usartSend_) {
      usartSend_(&send_data, sizeof(send_data));
    }
  }
};

}  // namespace tdtusart
#endif
