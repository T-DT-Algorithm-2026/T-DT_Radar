#include "usart.h"

tdtusart::CenterUsart* UsartPtr;

void on_exit([[maybe_unused]]int sig) {
  TDT_WARNING("Exit by Ctrl+C");
  UsartPtr->async_stop();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  signal(SIGINT, on_exit);
  tdtusart::CenterUsart usart("/dev/ttyUSB0",
                              921600);  // 进行串口的重映射与波特率设置
  UsartPtr = &usart;
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr ros2_node =
      rclcpp::Node::make_shared("roborts_usart_center_node");

  tdttoolkit::Time::Init(ros2_node->now());
  tdttoolkit::BaseBlackboard::Init("match_info", sizeof(tdttoolkit::MatchInfo));
  tdttoolkit::MatchInfo info;
  tdttoolkit::BaseBlackboard::Set("match_info", info);
  tdttoolkit::BaseBlackboard::Init("nav_status", sizeof(tdttoolkit::NavStatus));
  usart.init(ros2_node, argc, argv);
  usart.run();

  rclcpp::shutdown();
  return 0;
}