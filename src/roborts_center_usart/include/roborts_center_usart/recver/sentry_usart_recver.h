#ifndef __SENTRY_USART_RECVER_H
#define __SENTRY_USART_RECVER_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include "roborts_center_usart/base_usart.h"
#include "rclcpp/rclcpp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "vision_interface/msg/sentry2_radar.hpp"

namespace tdtusart {
class SentryUsartRecver : public BaseUsartRecver {
 public:
  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    sentry2RadarData = node->create_publisher<vision_interface::msg::Sentry2Radar>(
        "sentry2RadarData", 1);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/sentry", 10);
    marker.header.frame_id = "livox_frame";
    marker.ns = "sentry";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.lifetime = rclcpp::Duration(1,0);
  }
#pragma pack(1)
  struct SentryData {
    uint8_t header = 0xA5;
    uint8_t type = 2;

    int16_t self_x[6];
    int16_t self_y[6];

    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

  int GetStructLength() override { return sizeof(SentryData); }

  void ParseData(void *message) override {
    SentryData *sentry_data = (SentryData *)message;
    vision_interface::msg::Sentry2Radar sentry2Radar;
    for (int i = 0; i < 6; i++) {
      sentry2Radar.self_x[i] = sentry_data->self_x[i]/100.0;
      sentry2Radar.self_y[i] = sentry_data->self_y[i]/100.0;
    }
    
    sentry2RadarData->publish(sentry2Radar);
    marker.header.stamp = rclcpp::Clock().now();
    marker.pose.position.x = sentry2Radar.self_x[6];
    marker.pose.position.y = sentry2Radar.self_y[6];
    marker.pose.position.x+=4;
    marker.pose.position.y-=4;
    marker.pose.position.z = 0.1;
    marker_pub_->publish(marker);

    // TDT_INFO("Sentry Received&&Pub!");
  }
  visualization_msgs::msg::Marker marker;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<vision_interface::msg::Sentry2Radar>::SharedPtr sentry2RadarData;
};
}  // namespace tdtusart

#endif
