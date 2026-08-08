#ifndef __KEY_USART_SENDER_H
#define __KEY_USART_SENDER_H

#include <roborts_utils/base_msg.h>
#include <sys/types.h>

#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> // 新增：用于定时器的时间单位

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/password.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "vision_interface/msg/match_info.hpp"

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
    match_info_subscriber_ = node->create_subscription<vision_interface::msg::MatchInfo>(
        "match_info", 10,
        std::bind(&KeyUsartSender::MatchInfoCallback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)
  struct KeyData {
    uint8_t header = 0xA5;
    uint8_t type = 5;

    uint8_t radar_cmd;
    uint8_t cmd;
    uint8_t key[6];  

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::Password>::SharedPtr subscriber_;
  rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_info_subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;
  int16_t last_match_time_ = -200;
  uint8_t radar_cmd_ = 0;
  uint8_t key_[6] = {};
  bool has_key_ = false;
  bool match_started_ = false;
  bool dart_triggered_ = false;
  bool large_energy_triggered_ = false;

  void MatchInfoCallback(const vision_interface::msg::MatchInfo::SharedPtr msg)
  {
    if (msg->match_time == 419)
    {
      match_started_ = true;
    }
    else if (msg->match_time <= 0)
    {
      match_started_ = false;
    }
    if (msg->match_time > 0 && last_match_time_ <= 10)
    {
      radar_cmd_ = 0;
      dart_triggered_ = false;
      large_energy_triggered_ = false;
    }
    last_match_time_ = msg->match_time;//开始重置

    if (msg->large_energy_status == 0)
    {
      large_energy_triggered_ = false;
    }
    if (msg->match_time <= 10)
    {
      SendKeyData();
      return;
    }

    if (msg->dart_selected_target != 0 && !dart_triggered_ && radar_cmd_ < 2)
    {
      dart_triggered_ = true;
      radar_cmd_++;
    }
    else if (msg->large_energy_status == 1 && !large_energy_triggered_ && radar_cmd_ < 2)
    {
      large_energy_triggered_ = true;
      radar_cmd_++;
    }
    else if ((radar_cmd_ == 0 && msg->match_time <= 150) ||
             (radar_cmd_ == 1 && msg->match_time <= 80))
    {
      radar_cmd_++;
    }
    SendKeyData();
  }

  void Callback(const std::shared_ptr<const radio_interface::msg::Password> msg)
  {
    for (int i = 0; i < 6; i++)
    {
      key_[i] = msg->password[i];
    }
    has_key_ = true;
  }

  void SendKeyData()
  {
    if (!match_started_ || !has_key_ || !usartSend_)
    {
      return;
    }
    KeyData send_data;
    send_data.cmd = 2;
    send_data.radar_cmd = radar_cmd_;
    for (int i = 0; i < 6; i++)
    {
      send_data.key[i] = key_[i];
    }
    send_data.frame_id = frame_id_++;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
  }
};

}  // namespace tdtusart
#endif
