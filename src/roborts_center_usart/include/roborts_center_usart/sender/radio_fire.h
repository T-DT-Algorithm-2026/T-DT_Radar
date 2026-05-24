#ifndef __RADIO_FIRE_SENDER_H
#define __RADIO_FIRE_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/fire.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioFireSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ = node->create_subscription<radio_interface::msg::Fire>(
        "radio_fire",
        rclcpp::SensorDataQoS(),
        std::bind(&RadioFireSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)

  struct RadioFire {
    uint8_t header = 0xA5;
    uint8_t type = 7;

    uint16_t fire[5];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::Fire>::SharedPtr subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void Callback(const std::shared_ptr<const radio_interface::msg::Fire> msg) {
    RadioFire send_data;
    for (int i = 0; i < 5; i++) {
      send_data.fire[i] = msg->fire[i];
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
