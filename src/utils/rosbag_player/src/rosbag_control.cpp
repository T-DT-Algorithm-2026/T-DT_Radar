#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>

static struct termios old_tio;

void enable_raw_mode()
{
  struct termios new_tio;
  tcgetattr(STDIN_FILENO, &old_tio);
  new_tio = old_tio;
  new_tio.c_lflag &= ~(ICANON | ECHO); // disable canonical mode and echo
  new_tio.c_cc[VTIME] = 0;
  new_tio.c_cc[VMIN] = 0; // non-blocking
  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_mode()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rosbag_control_cpp");
  auto pub = node->create_publisher<std_msgs::msg::String>("/rosbag_player/control", 10);

  std::cout << "rosbag control (C++) started.\n";
  std::cout << "p = toggle pause/resume, q = quit\n";

  enable_raw_mode();

  bool exit = false;
  while (rclcpp::ok() && !exit) {
    // use select to wait for input with timeout
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    int ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
      char c = 0;
      ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n > 0) {
        if (c == 'p') {
          std_msgs::msg::String msg;
          msg.data = "toggle";
          pub->publish(msg);
          std::cout << "\nSent: toggle\n";
        } else if (c == 'q' || c == 3) { // 'q' or Ctrl-C
          std::cout << "\nQuit\n";
          exit = true;
          break;
        }
      }
    }

    rclcpp::spin_some(node);
  }

  restore_mode();
  rclcpp::shutdown();
  return 0;
}
