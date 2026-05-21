#ifndef __RADIO_STATE_SENDER_H
#define __RADIO_STATE_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioStateSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    
    usartSend_ = usartSend;

    // 直接启动 10ms 定时器，不再创建订阅者
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(13), 
        std::bind(&RadioStateSender::TimerCallback, this));
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
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void TimerCallback() {
    RadioState send_data;
    
    // ==========================================
    // 在这里设定你想测试的固定数据
    // ==========================================
    
    // 1. 测试占领信息 (例如：己方占领前哨站和堡垒)
    send_data.event_status.raw_data = 0;
    send_data.event_status.fields.outpost = 2;   // 2 代表己方
    send_data.event_status.fields.fortress = 2;
    send_data.event_status.fields.base = 1;      // 1 代表已占领
    
    // 2. 测试数值数据
    send_data.economy = 2000;
    send_data.uneconomy = 500;

    // 4. 更新基础信息
    // send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    send_data.frame_id = frame_id_++;

    // 5. 计算校验并发送
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    
    if (usartSend_) {
        usartSend_(&send_data, sizeof(send_data));
    }
    
    // 可选：在终端打印一下确认程序正在跑
    // TDT_INFO("Testing: Sending fixed Radio Data at 100Hz");
  }
};

}  // namespace tdtusart
#endif
