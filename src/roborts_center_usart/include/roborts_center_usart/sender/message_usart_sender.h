#ifndef __MESSAGE_USART_SENDER_H
#define __MESSAGE_USART_SENDER_H

#include <roborts_utils/base_msg.h>

#include <boost/asio.hpp>
#include <cstdint>
#include <rclcpp/qos.hpp>

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"
#include "vision_interface/msg/message.hpp"

// sender 向串口发送消息
namespace tdtusart {
class MessageUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    // (void)node;
    subscriber_ =
        node->create_subscription<vision_interface::msg::Message>(
            "MessagePub",       // topic_name,
            rclcpp::SensorDataQoS(),  // buff_size,
            std::bind(&MessageUsartSender::Callback, this,
                      std::placeholders::_1));
    usartSend_ = usartSend;
  }
#pragma pack(1)
  struct MessageData {
    uint8_t header = 0xA5;
    uint8_t type = 2;
    double time_stamp;  //视觉时间系下时间戳，用于给电控做时间同步用

    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // tdttoolkit::BaseCommunicator *communicator;
  rclcpp::Subscription<vision_interface::msg::Message>::SharedPtr
      subscriber_;

  std::function<bool(const void *, int)> usartSend_;

  int frame_id = 0;

  void Callback(
      const std::shared_ptr<const vision_interface::msg::Message>
          msg) {
    // TDT_WARNING("sending!");
    MessageData send_data;
    send_data.sender_id = msg->sender_id;
    send_data.receiver_id = msg->receiver_id;
    for (int i = 0; i < 30; i++) {
      send_data.user_data[i] = msg->user_data[i];
    }
    send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    send_data.frame_id = frame_id++;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    // TDT_INFO("Send Message");
  }
};

}  // namespace tdtusart
#endif