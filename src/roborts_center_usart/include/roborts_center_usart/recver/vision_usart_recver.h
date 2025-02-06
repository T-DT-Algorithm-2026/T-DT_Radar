// #ifndef __VISION_USART_RECVER_H
// #define __VISION_USART_RECVER_H

// #include <errno.h>
// #include <roborts_utils/base_msg.h>
// #include <roborts_utils/base_toolkit.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>

// #include <cstdint>
// #include <deque>
// #include <mutex>
// #include <vector>
// #include <vision_interface/msg/detail/usart_data__struct.hpp>

// #include "base_usart.h"
// #include "rclcpp/rclcpp.hpp"
// #include "roborts_utils/roborts_utils.h"
// #include "vision_interface/msg/usart_data.hpp"

// namespace tdtusart {
// class VisionUsartRecver : public BaseUsartRecver {
//  public:
//   void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
//     usartData = node->create_publisher<vision_interface::msg::UsartData>(
//         "visionUsartData", 1);
//   }
// #pragma pack(1)
//   struct VisionData {
//     uint8_t header = 0xA5;
//     uint8_t type = 3;

//     float yaw;
//     float pitch;
//     float time_stamp;

//     uint8_t enemycolor : 2;  // 0--Blue   1--error   2--Red
//     uint8_t buff_command : 2;  // 0--非打符模式   1--打符模式     2--反符模式
//     uint8_t building_command : 1;  // 0--非建筑击打模式    1--建筑击打模式
//     uint8_t buff_type : 1;
//     uint8_t lock_command : 1;
//     uint8_t spin_command : 1;
    

//     float nominal_bulletspeed;  // 标称弹速，即裁判系统限制的最高弹速
//     float bullet_speed;         // 实际弹速，即上一次发射的弹速

//     int16_t frame_id;
//     uint16_t CRC16CheckSum;
//   };
// #pragma pack()

//   int GetStructLength() override { return sizeof(VisionData); }

//   void ParseData(void *message) override {
//     VisionData *vision_data = (VisionData *)message;
//     vision_interface::msg::UsartData usrtPub;
//     TDT_INFO("Usart Received!");

//     usrtPub.recv_header.stamp =
//         tdttoolkit::Time::GetRosTimeByTime(vision_data->time_stamp * 1e3);

//     // TDT_DEBUG("usart time: %f", vision_data->time_stamp);
//     usrtPub.yaw = (vision_data->yaw * CV_PI) / 180.;  //弧度制
//     usrtPub.pitch = (vision_data->pitch * CV_PI) / 180.;

//     usrtPub.enemycolor = vision_data->enemycolor;

//     usrtPub.buff_command = vision_data->buff_command;
//     usrtPub.buff_type = vision_data->buff_type;

//     usrtPub.lock_command = vision_data->lock_command;
//     usrtPub.spin_command = vision_data->spin_command;
//     usrtPub.building_command = vision_data->building_command;

//     usrtPub.nominal_bulletspeed = vision_data->nominal_bulletspeed * 100;
//     usrtPub.bullet_speed =
//         vision_data->bullet_speed * 100;  // 评价是直接监视，速度不对直接找电控

//     usartData->publish(usrtPub);

//     // 如果usart 为空，输出错误信息
//     if (usartData == nullptr) {
//       TDT_ERROR("usartData is nullptr");
//     }
//   }

//   rclcpp::Publisher<vision_interface::msg::UsartData>::SharedPtr usartData;
// };
// }  // namespace tdtusart

// #endif
