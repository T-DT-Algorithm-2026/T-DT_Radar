#ifndef __USART_H
#define __USART_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <deque>
#include <mutex>
#include <vector>

#include "crc_tools.h"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"
namespace tdtusart {

class CenterUsart {
 public:
  void UsartOpen(std::string UsartPath, int baudrate = 921600);
  void UsartRestart();
  void UsartClose();

  void init(std::shared_ptr<rclcpp::Node> &node_, int argc, char **argv);

  void async_stop();
  void run();

  boost::asio::serial_port *serial_port_;                     // 设备句柄
  CenterUsart(std::string UsartPath, int baudrate = 921600);  // 串口初始化
  void RecvThread();                                          // 串口线程

 private:
  rclcpp::Node::SharedPtr ros2_node_ = nullptr;
  /**
   * @brief    串口接收数据
   *            要求启动后，在pc端发送ascii文件
   */
  // template <class T> int UsartPreRecv(T receive_message);
  int UsartPreRecv(volatile int *receive_message);

  int ReadUsart();

  bool usartSend(const void *data, int len);

  std::string usartPath;
  int baudrate;

  static const int UART_XMIT_SIZE =
      4096;  // Linux内核中指定的串口缓冲区大小，如果想加大可以修改对应代码重新编译内核

  struct RecvStatus {
    int recv_len = 0;
    bool ON_RECV_HEADER = true;
    bool ON_RECV_TYPE = true;
    bool ON_RECV_DATA = true;
    bool WRONG_TICK = false;
    unsigned char buff[UART_XMIT_SIZE];
    void reset() {
      recv_len = 0;
      ON_RECV_HEADER = true;
      ON_RECV_TYPE = true;
      ON_RECV_DATA = true;
    }
    void set_wrong_tick() {
      WRONG_TICK = true;
      ON_RECV_HEADER = true;
      ON_RECV_TYPE = true;
      ON_RECV_DATA = true;
    }
  } recv_buff;

  static const int frame_header = 0xA5;
  int cnt_recv_len = 0x7fffffff;
  std::mutex thread_locker;
  std::mutex send_locker;
  bool on_running = false;
};

}  // namespace tdtusart
#endif