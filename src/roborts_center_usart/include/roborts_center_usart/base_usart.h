#ifndef __BASE_USART_H
#define __BASE_USART_H
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tdtusart {
class BaseUsartRecver {
 public:
  virtual int GetStructLength() = 0;
  virtual void ParseData(void *message) = 0;
  virtual void init_communicator(std::shared_ptr<rclcpp::Node> &node) = 0;
};

class BaseUsartSender {
 public:
  virtual void init_communicator(
      std::shared_ptr<rclcpp::Node> &node,
      std::function<bool(const void *, int)> usartSend) = 0;
  // virtual void subscribe() = 0;
};
}  // namespace tdtusart

#endif