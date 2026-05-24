#ifndef __RADIO_HP_SENDER_H
#define __RADIO_HP_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/hp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioHpSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ = node->create_subscription<radio_interface::msg::Hp>(
        "radio_hp",
        rclcpp::SensorDataQoS(),
        std::bind(&RadioHpSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)

  struct RadioHp {
    uint8_t header = 0xA5;
    uint8_t type = 6;

    uint16_t hp[6];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::Hp>::SharedPtr subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void Callback(const std::shared_ptr<const radio_interface::msg::Hp> msg) {
    RadioHp send_data;
    for (int i = 0; i < 6; i++) {
      send_data.hp[i] = msg->hp[i];
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
