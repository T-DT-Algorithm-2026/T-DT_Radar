#ifndef __RADIO_HP_SENDER_H
#define __RADIO_HP_SENDER_H

#include <roborts_utils/base_msg.h>
#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> 

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {

class RadioHpSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    
    usartSend_ = usartSend;

    // 直接启动 10ms 定时器，不再创建订阅者
    // timer_ = node->create_wall_timer(
    //     std::chrono::milliseconds(10), 
    //     std::bind(&RadioHpSender::TimerCallback, this));
  }

#pragma pack(1)

  struct RadioHp {
    uint8_t header = 0xA5;
    uint8_t type = 6;

    uint16_t hp[6];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;
  int frame_id_ = 0;

  void TimerCallback() {
    RadioHp send_data;

    // 3. 测试数组数据 (全部赋相同的值或者递增值)
    for(int i = 0; i < 6; i++) {
      send_data.hp[i] = 1 + i; // 1000, 1001...
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
