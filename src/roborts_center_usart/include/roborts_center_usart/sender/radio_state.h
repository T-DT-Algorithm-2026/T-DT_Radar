#ifndef __RADIO_STATE_SENDER_H
#define __RADIO_STATE_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/state.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioStateSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ = node->create_subscription<radio_interface::msg::State>(
        "radio_state",
        rclcpp::SensorDataQoS(),
        std::bind(&RadioStateSender::Callback, this, std::placeholders::_1));
    usartSend_ = usartSend;
  }

#pragma pack(1)
  union BattlefieldEventData {
    uint16_t raw_data;
    struct {
      uint16_t supply_zone            : 1;  
      uint16_t center_highland        : 2;  
      uint16_t trapezoid_highland     : 1;  
      uint16_t fortress               : 2;  
      uint16_t outpost                : 2;  
      uint16_t base                   : 1;  
      uint16_t tunnel_enemy_front     : 1;  
      uint16_t tunnel_enemy_back      : 1;  
      uint16_t tunnel_self_front      : 1;  
      uint16_t tunnel_self_back       : 1;  
      uint16_t module_highland        : 1;  
      uint16_t module_fly_ramp_back   : 1;  
      uint16_t module_highway_upper   : 1;  
    } fields;
  };

  struct RadioState {
    uint8_t header = 0xA5;
    uint8_t type = 8;

    uint16_t economy;
    uint16_t uneconomy;

    BattlefieldEventData event_status;

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<radio_interface::msg::State>::SharedPtr subscriber_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void Callback(const std::shared_ptr<const radio_interface::msg::State> msg) {
    RadioState send_data;
    send_data.event_status.raw_data = 0;
    send_data.event_status.fields.supply_zone = msg->supply_area_occupied;
    send_data.event_status.fields.center_highland = msg->central_highland_status;
    send_data.event_status.fields.trapezoid_highland = msg->trapezoid_highland_occupied;
    send_data.event_status.fields.fortress = msg->sentry_patrol_status;
    send_data.event_status.fields.outpost = msg->front_sentry_gain_status;
    send_data.event_status.fields.base = msg->base_gain_occupied;
    send_data.event_status.fields.tunnel_enemy_front = msg->enemy_front_channel_detected;
    send_data.event_status.fields.tunnel_enemy_back = msg->enemy_back_channel_detected;
    send_data.event_status.fields.tunnel_self_front = msg->own_front_channel_detected;
    send_data.event_status.fields.tunnel_self_back = msg->own_back_channel_detected;
    send_data.event_status.fields.module_highland = msg->highland_upper_detected;
    send_data.event_status.fields.module_fly_ramp_back = msg->flying_slope_back_detected;
    send_data.event_status.fields.module_highway_upper = msg->highway_upper_detected;

    send_data.economy = msg->remaining_coin;
    send_data.uneconomy = msg->total_coin;
    send_data.frame_id = frame_id_++;

    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    if (usartSend_) {
      usartSend_(&send_data, sizeof(send_data));
    }
  }
};

}  // namespace tdtusart
#endif
