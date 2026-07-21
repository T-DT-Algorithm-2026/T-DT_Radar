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
    subscriber_ = node->create_subscription<radio_interface::msg::Buff>(
        "radio_buff",
        rclcpp::SensorDataQoS(),
        std::bind(&RadioBuffSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)

  struct RadioBuff {
    uint8_t header = 0xA5;
    uint8_t type = 9;

    uint8_t heal[5];
    uint16_t cooldown[5];
    uint8_t defence[5];
    uint8_t undefence[5];
    uint16_t attack[5];
    uint8_t sentry_posture;
    uint8_t main_posture[5];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::Buff>::SharedPtr subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void Callback(const std::shared_ptr<const radio_interface::msg::Buff> msg) {
    RadioBuff send_data;
    for (int i = 0; i < 5; i++) {
      send_data.heal[i] = msg->heal[i];
      send_data.cooldown[i] = msg->cooldown[i];
      send_data.defence[i] = msg->defence[i];
      send_data.undefence[i] = msg->undefence[i];
      send_data.attack[i] = msg->attack[i];
      send_data.main_posture[i] = msg->main_posture[i];
    }
    send_data.sentry_posture = msg->sentry_posture;
    send_data.frame_id = frame_id_++;

    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    if (usartSend_) {
      usartSend_(&send_data, sizeof(send_data));
    }
  }
};

}  // namespace tdtusart
#endif
