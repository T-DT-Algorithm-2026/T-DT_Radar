#ifndef __SENTRY_USART_RECVER_H
#define __SENTRY_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>
#include "roborts_center_usart/base_usart.h"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "vision_interface/msg/sentry2_radar.hpp"

namespace tdtusart {
class SentryUsartRecver : public BaseUsartRecver {
 public:
  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    sentry2RadarData = node->create_publisher<vision_interface::msg::Sentry2Radar>(
        "sentry2RadarData", 1);
  }
#pragma pack(1)
  struct SentryData {
    uint8_t header = 0xA5;
    uint8_t type = 2;

    uint8_t sentry_go_kill;

    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(SentryData); }

  void ParseData(void *message) override {
    SentryData *sentry_data = (SentryData *)message;
    vision_interface::msg::Sentry2Radar sentry2Radar;
    sentry2Radar.sentry_go_kill = sentry_data->sentry_go_kill;
    TDT_INFO("Sentry go kill: %u", static_cast<unsigned int>(sentry2Radar.sentry_go_kill));
    sentry2RadarData->publish(sentry2Radar);
  }

  rclcpp::Publisher<vision_interface::msg::Sentry2Radar>::SharedPtr sentry2RadarData;
};
}  // namespace tdtusart

#endif
