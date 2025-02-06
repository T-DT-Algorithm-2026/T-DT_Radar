#ifndef __DECISION_USART_SENDER_H
#define __DECISION_USART_SENDER_H

#include <boost/asio.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include <vision_interface/msg/radar_warn.hpp>

// sender 向串口发送消息
namespace tdtusart {
class DecisionUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node>& node,
      std::function<bool(const void*, int)> usartSend) override {

    subscriber_ =
        node->create_subscription<vision_interface::msg::RadarWarn>(
            "lidar_detect",  // topic_name,
            10,            
            std::bind(&DecisionUsartSender::Callback, this,
                      std::placeholders::_1));
    camera_subscriber_ =
        node->create_subscription<vision_interface::msg::RadarWarn>(
            "hero_state",  // topic_name,
            10,            
            std::bind(&DecisionUsartSender::camera_callback, this,
                      std::placeholders::_1));
    usartSend_ = usartSend;
  }
#pragma pack(1)
  struct DecisionData {
    uint8_t header = 0xA5;
    uint8_t type = 1;  //第二种UsartSender

    uint8_t fly_state;
    uint8_t dart_state;
    uint8_t hero_state;

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // tdttoolkit::BaseCommunicator *communicator;
  rclcpp::Subscription<vision_interface::msg::RadarWarn>::SharedPtr
      subscriber_;
      rclcpp::Subscription<vision_interface::msg::RadarWarn>::SharedPtr
      camera_subscriber_;
    DecisionData send_data;

  std::function<bool(const void*, int)> usartSend_;

  int frame_id = 0;

  void camera_callback(const vision_interface::msg::RadarWarn::SharedPtr msg) {
    send_data.hero_state = msg->hero_state;

    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t*)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
  }

  void Callback(
      const std::shared_ptr<const vision_interface::msg::RadarWarn>
          msg) {

    
    send_data.fly_state = msg->fly_state;
    send_data.dart_state = msg->dart_state;

    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t*)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    // TDT_INFO("Send Vision Data %f %f ", send_data.yaw, send_data.pitch);
  }
};

}  // namespace tdtusart
#endif