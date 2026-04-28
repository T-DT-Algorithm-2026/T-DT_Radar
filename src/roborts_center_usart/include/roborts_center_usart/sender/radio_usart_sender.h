// #ifndef __RADIO_USART_SENDER_H
// #define __RADIO_USART_SENDER_H

// #include <roborts_utils/base_msg.h>

// #include <boost/asio.hpp>
// #include <rclcpp/qos.hpp>

// #include "base_usart.h"
// #include "crc_tools.h"
// #include "roborts_utils/roborts_utils.h"
// #include "usart.h"
// #include "vision_interface/msg/radio.hpp" // 确保这个路径正确

// namespace tdtusart {

// class RadioUsartSender : public BaseUsartSender {
//  public:
//   void init_communicator(
//       std::shared_ptr<rclcpp::Node> &node,
//       std::function<bool(const void *, int)> usartSend) override {
      
//     // 修正了这里的订阅者类型，使其与你的回调函数及 ROS 消息一致
//     subscriber_ = node->create_subscription<vision_interface::msg::Radio>(
//             "radio",
//             rclcpp::SensorDataQoS(),
//             std::bind(&RadioUsartSender::Callback, this, std::placeholders::_1));
            
//     usartSend_ = usartSend;
//   }

// #pragma pack(1)
//   // 核心：占领信息位域，刚好占用 2 字节 (16 bit)
//   union BattlefieldEventData {
//     uint16_t raw_data;
//     struct {
//       uint16_t supply_zone            : 1;  
//       uint16_t center_highland        : 2;  
//       uint16_t trapezoid_highland     : 1;  
//       uint16_t fortress               : 2;  
//       uint16_t outpost                : 2;  
//       uint16_t base                   : 1;  
//       uint16_t tunnel_enemy_front     : 1;  
//       uint16_t tunnel_enemy_back      : 1;  
//       uint16_t tunnel_self_front      : 1;  
//       uint16_t tunnel_self_back       : 1;  
//       uint16_t module_highland        : 1;  
//       uint16_t module_fly_ramp_back   : 1;  
//       uint16_t module_highway_upper   : 1;  
//     } fields;
//   };

//   // 完整的串口数据包结构体
//   struct RadioPacket {
//     uint8_t header = 0xA5;
//     uint8_t type = 6;

//     // --- 依据你的 ROS 自定义消息添加的数据段 ---
//     uint16_t hp[6];
//     uint16_t fire[5];
    
//     uint16_t economy;
//     uint16_t uneconomy;

//     // 占领信息（压缩为 2 字节）
//     BattlefieldEventData event_status;

//     uint8_t heal[5];
//     uint16_t cooldown[5];
//     uint8_t defence[5];
//     uint8_t undefence[5];
//     uint16_t attack[5];

//     float time_stamp;  // 时间戳
//     int16_t frame_id;

//     // 校验和
//     uint16_t CRC16CheckSum;
//   };
// #pragma pack()

//  private:
//   // 修正为 Radio 类型
//   rclcpp::Subscription<vision_interface::msg::Radio>::SharedPtr subscriber_;
//   std::function<bool(const void *, int)> usartSend_;
//   int frame_id = 0;

//   void Callback(const std::shared_ptr<const vision_interface::msg::Radio> msg) {
//     RadioPacket send_data;
    
//     // 1. 初始化联合体（清零，防止内存残留的垃圾数据）
//     send_data.event_status.raw_data = 0;

//     // 2. 映射占领信息位域
//     send_data.event_status.fields.supply_zone          = msg->supply_zone;
//     send_data.event_status.fields.center_highland      = msg->center_highland;
//     send_data.event_status.fields.trapezoid_highland   = msg->trapezoid_highland;
//     send_data.event_status.fields.fortress             = msg->fortress;
//     send_data.event_status.fields.outpost              = msg->outpost;
//     send_data.event_status.fields.base                 = msg->base;
//     send_data.event_status.fields.tunnel_enemy_front   = msg->tunnel_enemy_front;
//     send_data.event_status.fields.tunnel_enemy_back    = msg->tunnel_enemy_back;
//     send_data.event_status.fields.tunnel_self_front    = msg->tunnel_self_front;
//     send_data.event_status.fields.tunnel_self_back     = msg->tunnel_self_back;
//     send_data.event_status.fields.module_highland      = msg->module_highland;
//     send_data.event_status.fields.module_fly_ramp_back = msg->module_fly_ramp_back;
//     send_data.event_status.fields.module_highway_upper = msg->module_highway_upper;

//     // 3. 映射标量数据
//     send_data.economy = msg->economy;
//     send_data.uneconomy = msg->uneconomy;

//     // 4. 修复越界 Bug：分别映射大小为 6 和大小为 5 的数组
//     for(int i = 0; i < 6; i++) {
//       send_data.hp[i] = msg->hp[i];
//     }

//     for(int i = 0; i < 5; i++) {
//       send_data.fire[i] = msg->fire[i];
//       send_data.heal[i] = msg->heal[i];
//       send_data.cooldown[i] = msg->cooldown[i];
//       send_data.defence[i] = msg->defence[i];
//       send_data.undefence[i] = msg->undefence[i];
//       send_data.attack[i] = msg->attack[i];
//     }

//     send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
//     send_data.frame_id = frame_id++;

//     // 5. 计算校验和并发送
//     CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
//     usartSend_(&send_data, sizeof(send_data));
//     TDT_INFO("Send Radar Data");
//   }
// };

// }  // namespace tdtusart
// #endif

#ifndef __RADIO_USART_SENDER_H
#define __RADIO_USART_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    
    usartSend_ = usartSend;

    // 直接启动 10ms 定时器，不再创建订阅者
    // timer_ = node->create_wall_timer(
    //     std::chrono::milliseconds(10), 
    //     std::bind(&RadioUsartSender::TimerCallback, this));
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

  struct RadioPacket {
    uint8_t header = 0xA5;
    uint8_t type = 6;

    uint16_t hp[6];
    uint16_t fire[5];

    uint16_t economy;
    uint16_t uneconomy;

    BattlefieldEventData event_status;

    uint8_t heal[5];
    uint16_t cooldown[5];
    uint8_t defence[5];
    uint8_t undefence[5];
    uint16_t attack[5];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void TimerCallback() {
    RadioPacket send_data;
    
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

    // 3. 测试数组数据 (全部赋相同的值或者递增值)
    for(int i = 0; i < 6; i++) {
      send_data.hp[i] = 1000 + i; // 1000, 1001...
    }

    for(int i = 0; i < 5; i++) {
      send_data.fire[i] = 50;
      send_data.heal[i] = 10;
      send_data.cooldown[i] = 0;
      send_data.defence[i] = 100;
      send_data.undefence[i] = 0;
      send_data.attack[i] = 150;
    }

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