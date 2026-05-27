#ifndef __RADAR_USART_SENDER_H
#define __RADAR_USART_SENDER_H

#include <roborts_utils/base_msg.h>

#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "vision_interface/msg/radar2_sentry.hpp"

// sender 向串口发送消息
namespace tdtusart {
class RadarUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    subscriber_ =
        node->create_subscription<vision_interface::msg::Radar2Sentry>(
            "Radar2Sentry",       // topic_name,
            rclcpp::SensorDataQoS(),
            std::bind(&RadarUsartSender::Callback, this,
                      std::placeholders::_1));
    usartSend_ = usartSend;
  }
#pragma pack(1)
  struct Radar2SentryData {
    uint8_t header = 0xA5;
    uint8_t type = 3;

    uint16_t radar_enemy_x[6];
    uint16_t radar_enemy_y[6];
    uint16_t radar_ally_x[6];
    uint16_t radar_ally_y[6];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::Subscription<vision_interface::msg::Radar2Sentry>::SharedPtr
      subscriber_;

  std::function<bool(const void *, int)> usartSend_;

  void Callback(
      const std::shared_ptr<const vision_interface::msg::Radar2Sentry>
          msg) {
    // TDT_WARNING("sending!");
    Radar2SentryData send_data;
    for (int i = 0; i < 6; i++) {
      send_data.radar_enemy_x[i] = msg->radar_enemy_x[i]*100;
      send_data.radar_enemy_y[i] = msg->radar_enemy_y[i]*100;
    }
    for(int i = 0; i < 6; i++) {
      send_data.radar_ally_x[i] = msg->radar_ally_x[i]*100;
      send_data.radar_ally_y[i] = msg->radar_ally_y[i]*100;
    }
    //         std::cout<<"ally"<<send_data.radar_ally_x[0]<<" "<<send_data.radar_ally_y[0]<<std::endl;
    // std::cout<<"enemy"<<send_data.radar_enemy_x[0]<<" "<<send_data.radar_enemy_y[0]<<std::endl;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));

    usartSend_(&send_data, sizeof(send_data));

    TDT_INFO("Send Radar Data");
  }
};

}  // namespace tdtusart
#endif