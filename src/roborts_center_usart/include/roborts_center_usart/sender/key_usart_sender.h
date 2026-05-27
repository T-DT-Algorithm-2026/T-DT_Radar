#ifndef __KEY_USART_SENDER_H
#define __KEY_USART_SENDER_H

#include <roborts_utils/base_msg.h>

#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> // 新增：用于定时器的时间单位

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/password.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {
class KeyUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ = node->create_subscription<radio_interface::msg::Password>(
        "key_usart_sender",
        rclcpp::SensorDataQoS(),
        std::bind(&KeyUsartSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)
  struct KeyData {
    uint8_t header = 0xA5;
    uint8_t type = 5;

    uint8_t cmd;
    uint8_t key[6];  

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::Password>::SharedPtr subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void Callback(const std::shared_ptr<const radio_interface::msg::Password> msg) {
    KeyData send_data;
    send_data.cmd = 2;
    for (int i = 0; i < 6; i++) {
      send_data.key[i] = msg->password[i];
      // send_data.key[i] = msg->password[5 - i];
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
