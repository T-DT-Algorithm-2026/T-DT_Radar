#include "usart.h"

#include <roborts_utils/base_msg.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "usart_info_summary.h"
namespace tdtusart {

CenterUsart::CenterUsart(std::string UsartPath, int baudrate) {
  this->usartPath = UsartPath;
  this->baudrate = baudrate;
  UsartOpen(this->usartPath, this->baudrate);
}

void CenterUsart::UsartOpen(std::string UsartPath, int baudrate) {
  (void) UsartPath;
  using namespace boost::asio;
  boost::asio::io_service iosev;
OPEN_SERIAL:
  try {
    serial_port_ = new serial_port(iosev, usartPath);
  } catch (boost::system::system_error &e) {
    TDT_ERROR("Open Usart Error: %s", e.what());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    goto OPEN_SERIAL;
  }
  serial_port_->set_option(serial_port::baud_rate(baudrate));
  serial_port_->set_option(
      serial_port::flow_control(serial_port::flow_control::none));
  serial_port_->set_option(serial_port::parity(serial_port::parity::none));
  serial_port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  serial_port_->set_option(serial_port::character_size(8));
}

int CenterUsart::ReadUsart() {
  if (serial_port_->is_open()) {
    while (true) {
      try {
        if (recv_buff.ON_RECV_HEADER) {
          if (recv_buff.recv_len < 1) {
            int recv_len = boost::asio::read(
                *serial_port_, boost::asio::buffer(recv_buff.buff, 1));
            if (recv_len <= 0) {
              TDT_ERROR("No Data");
              return -3;  // Read No Data
            }
            recv_buff.recv_len = recv_len;
          }
          if (recv_buff.buff[0] == frame_header && (!recv_buff.WRONG_TICK)) {
            recv_buff.ON_RECV_HEADER = false;
          } else {
            recv_buff.WRONG_TICK = false;
            bool find_header = false;
            for (int i = 1; i < recv_buff.recv_len; i++) {
              if (recv_buff.buff[i] == frame_header) {
                recv_buff.recv_len = recv_buff.recv_len - i;
                memcpy(recv_buff.buff, recv_buff.buff + i, recv_buff.recv_len);
                recv_buff.ON_RECV_HEADER = false;
                recv_buff.ON_RECV_TYPE = true;
                recv_buff.ON_RECV_DATA = true;
                find_header = true;
                break;
              }
            }
            if (!find_header) {
              recv_buff.reset();
              recv_buff.ON_RECV_HEADER = true;
              recv_buff.ON_RECV_TYPE = true;
              recv_buff.ON_RECV_DATA = true;
            }
            continue;
          }
        }

        if (recv_buff.ON_RECV_TYPE) {
          if (recv_buff.recv_len < 2) {
            int recv_len = boost::asio::read(
                *serial_port_, boost::asio::buffer(recv_buff.buff + 1, 1));
            if (recv_len <= 0) {
              TDT_ERROR("No Data");
              return -3;
            }
            recv_buff.recv_len = 1 + recv_len;
          }
          if (recv_buff.buff[1] >= 0x01 &&
              recv_buff.buff[1] <= shared_data::DataRecverNum) {
            recv_buff.ON_RECV_TYPE = false;
          } else {
            recv_buff.set_wrong_tick();
            continue;
          }
        }

        if (recv_buff.ON_RECV_DATA) {
          int expected_recv_len =
              shared_data::DataRecver[recv_buff.buff[1]]->GetStructLength();
          if (recv_buff.recv_len < expected_recv_len) {
            int recv_len = boost::asio::read(
                *serial_port_,
                boost::asio::buffer(recv_buff.buff + recv_buff.recv_len,
                                    expected_recv_len - recv_buff.recv_len));
            if (recv_len <= 0) {
              TDT_ERROR("No Data");
              return -3;
            }
            recv_buff.recv_len = recv_buff.recv_len + recv_len;
          }
          if (recv_buff.recv_len < expected_recv_len) {
            TDT_ERROR("Read Length Error");
            return -2;  // Read Length Error
          }
          if (CRC::VerifyCRC16CheckSum(recv_buff.buff, expected_recv_len)) {
            recv_buff.ON_RECV_DATA = false;
          } else {
            recv_buff.set_wrong_tick();
            char tmp[1024];
            memcpy(tmp, recv_buff.buff, expected_recv_len);
            CRC::AppendCRC16CheckSum((uint8_t*)&(tmp), expected_recv_len);
            TDT_ERROR("CRC Error: recv CRC value=%d,%d, expected CRC value=%d,%d",
                      recv_buff.buff[expected_recv_len - 2], recv_buff.buff[expected_recv_len - 1],
                      tmp[expected_recv_len - 2], tmp[expected_recv_len - 1]
                     );

            for(int i = 0;i<expected_recv_len;i++){
              printf("%02x ",recv_buff.buff[i]);
            }
            
            printf("\n");
            return -1;  // CRC Error
          }
        }
        return recv_buff.buff[1];
      } catch (...) {
        return -3;
        TDT_ERROR("333");
      }
    }
  } else {
    return -4;  // Usart Offline
    TDT_ERROR("Usart Offline");
  }
}

bool CenterUsart::usartSend(const void *data, int len) {
  if (serial_port_->is_open()) {
    std::unique_lock<std::mutex> locker(send_locker);
    try {
      boost::asio::write(*serial_port_, boost::asio::buffer(data, len));
    } catch (...) {
      locker.unlock();
      return false;
    }
    locker.unlock();
    return true;
  } else {
    return false;
  }
}

void CenterUsart::RecvThread() {
  double last_recv_time = tdttoolkit::Time::GetTimeNow();
  while (on_running) {
    thread_locker.lock();
    int ret = ReadUsart();
    thread_locker.unlock();
    if (ret == -3) {
      double cnt_time = tdttoolkit::Time::GetTimeNow();
      if (fabs(cnt_time - last_recv_time) > 1e6) {
        last_recv_time = cnt_time;
        TDT_WARNING("串口获取超时,重启串口");
        UsartRestart();
      }
      usleep(100);  // 两次主动获取串口信息之间延时，单位us
    } else {
      last_recv_time = tdttoolkit::Time::GetTimeNow();
    }
    if (ret == -1) {
      // TDT_WARNING("CRC校验失败");
      // usleep(10);
    }
    if (ret == -4) {
      TDT_WARNING("串口离线,重启串口");
      UsartRestart();
      usleep(100);
    }
    if (ret > 0) {
      shared_data::DataRecver[ret]->ParseData(recv_buff.buff);
      recv_buff.reset();
    }
  }
}

void CenterUsart::UsartRestart() { UsartOpen(this->usartPath, this->baudrate); }

void CenterUsart::UsartClose() {
  serial_port_->close();
  on_running = false;
}

void CenterUsart::run() {
  on_running = true;
  // RecvThread();
  auto recv_thread = std::thread(&CenterUsart::RecvThread, this);
  recv_thread.detach();
  rclcpp::spin(ros2_node_);
}

void CenterUsart::init(std::shared_ptr<rclcpp::Node> &node_, int argc,
                       char **argv) {
  (void)argc;
  (void)argv;
  ros2_node_ = node_;
  for (int i = 1; i <= shared_data::DataSenderNum; i++) {
    shared_data::DataSender[i]->init_communicator(
        node_, std::bind(&CenterUsart::usartSend, this, std::placeholders::_1,
                         std::placeholders::_2));
    // shared_data::DataSender[i]->subscribe();
  }

  for (int i = 1; i <= shared_data::DataRecverNum; i++) {
    shared_data::DataRecver[i]->init_communicator(node_);
    // shared_data::DataSender[i]->subscribe();
  }
}

void CenterUsart::async_stop() {
  on_running = false;
  try {
    serial_port_->close();
  } catch (...) {
    TDT_ERROR("串口关闭失败");
  }
}

}  // namespace tdtusart