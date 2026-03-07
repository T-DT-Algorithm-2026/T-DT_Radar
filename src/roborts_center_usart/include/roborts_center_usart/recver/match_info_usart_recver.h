#ifndef __MATCH_INFO_USART_RECVER_H
#define __MATCH_INFO_USART_RECVER_H

#include <errno.h>
#include <roborts_utils/base_blackboard.h>
#include <roborts_utils/base_class.h>
#include <roborts_utils/base_toolkit.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <deque>
#include <mutex>
#include <rclcpp/client.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <vector>

#include "base_usart.h"
#include "roborts_utils/roborts_utils.h"
#include "vision_interface/msg/match_info.hpp"

namespace tdtusart {
class MatchInfoUsartRecver : public BaseUsartRecver {
 public:
// clang-format off
#pragma pack(1)
  struct MatchInfo { // 1s 一次
    uint8_t header = 0xA5;
    uint8_t type = 1;
    
    uint8_t self_color=1;
    int16_t match_time = -200; // 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100 若未连接至裁判系统发送-200
    uint8_t robot_hp[16];
    uint8_t marks[6];
    uint8_t ultimate;
    uint32_t eventType;

    float time_stamp;
    uint16_t frame_id;
    uint16_t CRC16CheckSum;
  };
// clang-format on
#pragma pack()

  int GetStructLength() override { return sizeof(MatchInfo); }

  void ParseData(void *message) override {
    auto match_info = std::make_shared<vision_interface::msg::MatchInfo>();
    match_info->self_color = ((MatchInfo *)message)->self_color;
    match_info->match_time = ((MatchInfo *)message)->match_time;
    for (int i = 0; i < 16; i++) {
      match_info->robot_hp[i] = ((MatchInfo *)message)->robot_hp[i];
    }
    for (int i = 0; i < 6; i++) {
      match_info->marks[i] = ((MatchInfo *)message)->marks[i];
    }
    match_info->ultimate = ((MatchInfo *)message)->ultimate;
    match_info->eventtype=((MatchInfo *)message)->eventType;
    matchInfoPub->publish(*match_info);
      TDT_INFO("matchInfoPub Received&&Pub!");

  }

  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    matchInfoPub = node->create_publisher<vision_interface::msg::MatchInfo>("match_info", 1);
  }
      rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr matchInfoPub;

};
}  // namespace tdtusart

#endif
