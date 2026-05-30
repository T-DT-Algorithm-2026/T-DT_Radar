#ifndef __DECISION_USART_SENDER_H
#define __DECISION_USART_SENDER_H

#include <array>
#include <boost/asio.hpp>
#include <chrono>

#include "base_usart.h"
#include "crc_tools.h"
#include "radio_interface/msg/hp.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart.h"

// sender 向串口发送消息
namespace tdtusart {
class DecisionUsartSender : public BaseUsartSender {
 public:
  void init_communicator(
      std::shared_ptr<rclcpp::Node>& node,
      std::function<bool(const void*, int)> usartSend) override {

    usartSend_ = usartSend;
    hp_subscriber_ = node->create_subscription<radio_interface::msg::Hp>(
        "radio_hp",
        rclcpp::SensorDataQoS(),
        std::bind(&DecisionUsartSender::HpCallback, this,
                  std::placeholders::_1));
    timer_ = node->create_wall_timer(
        kSendInterval,
        std::bind(&DecisionUsartSender::TimerCallback, this));
  }
#pragma pack(1)
  struct DecisionData {
    uint8_t header = 0xA5;
    uint8_t type = 1;  //第二种UsartSender

    // 发送给哨兵的无敌车状态数组:
    // invincible_state[i] == 1 表示 radio_hp.hp[i] 对应车辆处于无敌状态
    // invincible_state[i] == 0 表示该车辆可以正常打
    uint8_t invincible_state[6];

    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
#pragma pack()

 private:
  using Clock = std::chrono::steady_clock;

  // 每辆车单独维护一份血量历史和无敌计时状态
  struct HpState {
    bool has_hp = false;
    uint16_t last_hp = 0;
    int low_after_zero_count = 0;
    Clock::time_point last_hp_time;
    Clock::time_point invincible_start_time;
    Clock::time_point invincible_until;
  };

  static constexpr int kRobotCount = 6;
  static constexpr uint16_t kInvincibleHpMax = 50;
  static constexpr int kLowConfirmFrames = 2;
  static constexpr auto kSendInterval = std::chrono::milliseconds(100);  // 10Hz发送
  static constexpr auto kHpFreshTimeout = std::chrono::milliseconds(350);  // 血量丢包保护
  static constexpr auto kInvincibleDuration = std::chrono::seconds(30);  // 无敌持续时间

  // tdttoolkit::BaseCommunicator *communicator;
  rclcpp::Subscription<radio_interface::msg::Hp>::SharedPtr hp_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<HpState, kRobotCount> hp_states_;
  // 对外发送的无敌状态, 1为无敌, 0为可打
  std::array<uint8_t, kRobotCount> invincible_state_{};

  std::function<bool(const void*, int)> usartSend_;

  int frame_id_ = 0;

  // 收到一帧血量后, 逐车更新无敌判断, 并立即发送最新结果
  void HpCallback(const std::shared_ptr<const radio_interface::msg::Hp> msg) {
    auto now = Clock::now();
    for (int i = 0; i < kRobotCount; i++) {
      UpdateRobotHp(i, msg->hp[i], now);
    }
    SendLatestData(now);
  }

  void TimerCallback() {
    SendLatestData(Clock::now());
  }

  // 无敌判定:
  // 1. 新鲜的上一帧为0血, 当前血量跳到1~50, 开始确认
  // 2. 连续确认到2帧1~50后才认为无敌, 防止单帧误报
  // 3. 无敌期间看到0血、>50血、超时或血量断流, 都立刻清掉
  void UpdateRobotHp(int index, uint16_t hp, Clock::time_point now) {
    HpState& state = hp_states_[index];
    bool last_hp_is_fresh =
        state.has_hp && now - state.last_hp_time <= kHpFreshTimeout;
    bool rose_from_zero = last_hp_is_fresh && state.last_hp == 0;
    bool confirming_low_hp =
        last_hp_is_fresh && state.low_after_zero_count > 0;

    if (!last_hp_is_fresh) {
      ClearInvincible(index);
    }

    // 0血不是无敌, 只是等待下一次从0跳到低血量
    if (hp == 0) {
      ClearInvincible(index);
    // 血量大于50说明无敌状态已经解除, 宁可继续打也不误报无敌
    } else if (hp > kInvincibleHpMax) {
      ClearInvincible(index);
    // 当前还没判无敌, 且满足从0血跳到1~50或正在确认低血量
    } else if (invincible_state_[index] == 0 &&
               (rose_from_zero || confirming_low_hp)) {
      if (state.low_after_zero_count == 0) {
        state.invincible_start_time = now;
      }
      state.low_after_zero_count++;
      // 连续多帧确认后, 开始30秒无敌计时
      if (state.low_after_zero_count >= kLowConfirmFrames) {
        invincible_state_[index] = 1;
        state.invincible_until =
            state.invincible_start_time + kInvincibleDuration;
        state.low_after_zero_count = 0;
      }
    }

    // 30秒到了自动解除
    if (invincible_state_[index] == 1 && now >= state.invincible_until) {
      ClearInvincible(index);
    }

    state.has_hp = true;
    state.last_hp = hp;
    state.last_hp_time = now;
  }

  // 定时发送前刷新状态, 防止血量话题断流后还一直给哨兵报无敌
  void RefreshInvincibleState(Clock::time_point now) {
    for (int i = 0; i < kRobotCount; i++) {
      HpState& state = hp_states_[i];
      if (!state.has_hp) {
        continue;
      }
      if (now - state.last_hp_time > kHpFreshTimeout) {
        ClearInvincible(i);
      } else if (invincible_state_[i] == 1 && now >= state.invincible_until) {
        ClearInvincible(i);
      }
    }
  }

  // 清掉无敌状态和低血确认计数
  void ClearInvincible(int index) {
    invincible_state_[index] = 0;
    hp_states_[index].low_after_zero_count = 0;
  }

  // 按固定协议打包并发送给哨兵
  void SendLatestData(Clock::time_point now) {
    if (!usartSend_) {
      return;
    }
    RefreshInvincibleState(now);

    DecisionData send_data;
    for (int i = 0; i < kRobotCount; i++) {
      send_data.invincible_state[i] = invincible_state_[i];
    }
    send_data.frame_id = frame_id_++;
    std::cout<<"Send Invincible State: ["<< (int)send_data.invincible_state[0] << ", "
             << (int)send_data.invincible_state[1] << ", "
             << (int)send_data.invincible_state[2] << ", "
             << (int)send_data.invincible_state[3] << ", "
             << (int)send_data.invincible_state[4] << ", "
             << (int)send_data.invincible_state[5] << "]"
             << std::endl;
    CRC::AppendCRC16CheckSum((uint8_t*)&(send_data), sizeof(send_data));
    usartSend_(&send_data, sizeof(send_data));
    // TDT_INFO("Send Vision Data %f %f ", send_data.yaw, send_data.pitch);
  }
};

}  // namespace tdtusart
#endif
