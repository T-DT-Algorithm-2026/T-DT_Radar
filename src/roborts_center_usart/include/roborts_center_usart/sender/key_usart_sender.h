#ifndef __KEY_USART_SENDER_H
#define __KEY_USART_SENDER_H

#include <roborts_utils/base_msg.h>

#include <boost/asio.hpp>
#include <rclcpp/qos.hpp>
#include <chrono> // 新增：用于定时器的时间单位

#include "base_usart.h"
#include "crc_tools.h"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

namespace tdtusart {
class KeyUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) override {
    
    // 保存发送函数句柄
    usartSend_ = usartSend;

    // 关键修改：创建一个定时器，主动、周期性地触发发送函数。
    // 这里设置为 10 毫秒 (100Hz) 发送一次，你可以根据电控需求修改这个频率
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&KeyUsartSender::TimerCallback, this));
  }

#pragma pack(1)
  struct KeyData {
    uint8_t header = 0xA5;
    uint8_t type = 5;

    uint8_t cmd;
    uint8_t key[6];  

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  // 将原来的 subscriber_ 替换为 timer_
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<bool(const void *, int)> usartSend_;

  // 新增：定时器回调函数，不需要传入 msg 参数
  void TimerCallback() {
    KeyData send_data;
    send_data.cmd = 2; // 这里你可以设置一个固定的命令字，或者根据需要修改为动态值
    // 这里同样可以设置 key 数组的值，或者保持为默认的0，根据你的协议需求来定
    send_data.key[0] = '5';
    send_data.key[1] = '7';
    send_data.key[2] = 'A';
    send_data.key[3] = 'h';
    send_data.key[4] = 'P';
    send_data.key[5] = 'P';// 示例：设置第一个按键状态为1，表示按下

    
    // send_data.time_stamp = tdttoolkit::Time::GetTimeNow() / 1e3;
    CRC::AppendCRC16CheckSum((uint8_t *)&(send_data), sizeof(send_data));
    
    // // 打印调试信息
    // std::cout << "Gimbal Auto Send -> yaw: " << send_data.yaw 
    //           << ", pitch: " << send_data.pitch 
    //           << ", is_fire: " << send_data.is_fire << "\r" << std::flush;
    // std::cout<<"single"<<std::endl;

    // 执行串口发送
    if (usartSend_) {
        usartSend_(&send_data, sizeof(send_data));
    }
  }
};

}  // namespace tdtusart
#endif