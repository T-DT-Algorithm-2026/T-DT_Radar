#pragma once

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>

namespace tdt_lock {

// 角度卡尔曼的配置。角度统一使用 rad，角速度统一使用 rad/s。
// 节点侧以像素或角度定义的固定参数会在初始化时换算后写入这里。
struct AngleKalmanConfig
{
    // 相机测得的绝对目标 yaw/pitch 的标准差，对应测量噪声矩阵 R。
    double measurement_std_yaw_rad = 2.0e-4;
    double measurement_std_pitch_rad = 2.0e-4;

    // 为保持与 CV 配置接口一致沿用该名称；在 CA 中表示连续白噪声角加加速度强度，单位 rad^2/s^5。
    double angular_acceleration_noise = 0.05;

    // 第一次建立滤波器时角速度和角加速度的不确定度。
    double initial_velocity_std_rad_s = 10.0 * 3.14159265358979323846 / 180.0;
    double initial_acceleration_std_rad_s2 = 100.0 * 3.14159265358979323846 / 180.0;
};

// 标准线性卡尔曼的最小实现。
// 在本文件中的实际状态为 [yaw, yaw_rate, yaw_acc, pitch, pitch_rate, pitch_acc]，
// 测量为 [yaw, pitch]。模板保留了矩阵维度，便于检查矩阵运算是否匹配。
template <int StateDim, int MeasurementDim>
class StandardKF
{
public:
    using State = Eigen::Matrix<double, StateDim, 1>;
    using Measurement = Eigen::Matrix<double, MeasurementDim, 1>;
    using StateCov = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasurementCov = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
    using TransitionMatrix = Eigen::Matrix<double, StateDim, StateDim>;
    using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, StateDim>;

    void init(const cv::Point2d& angles, double yaw_variance, double pitch_variance,
              double velocity_variance, double acceleration_variance)
    {
        // 初始角度直接使用第一帧测量，初始角速度和角加速度设为 0。
        state_.setZero();
        state_(0) = angles.x;
        state_(3) = angles.y;

        // P 的对角线表示各状态初始方差，速度和加速度初值未知，因此给出较大方差。
        state_cov_.setZero();
        state_cov_(0, 0) = yaw_variance;
        state_cov_(1, 1) = velocity_variance;
        state_cov_(2, 2) = acceleration_variance;
        state_cov_(3, 3) = pitch_variance;
        state_cov_(4, 4) = velocity_variance;
        state_cov_(5, 5) = acceleration_variance;
    }

    void predict(const TransitionMatrix& transition, const StateCov& process_noise)
    {
        // 时间预测：x(k|k-1) = F*x(k-1|k-1)
        // 协方差预测：P(k|k-1) = F*P*F^T + Q
        state_ = transition * state_;
        state_cov_ = transition * state_cov_ * transition.transpose() + process_noise;
    }

    void update(const Measurement& measurement, const MeasurementMatrix& observation,
                const MeasurementCov& measurement_noise)
    {
        // 创新协方差 S = H*P*H^T + R，卡尔曼增益 K = P*H^T*S^-1。
        MeasurementCov innovation_cov =
            observation * state_cov_ * observation.transpose() + measurement_noise;
        Eigen::Matrix<double, StateDim, MeasurementDim> gain =
            state_cov_ * observation.transpose() * innovation_cov.inverse();

        // 测量校正：x(k|k) = x(k|k-1) + K*(z - H*x(k|k-1))。
        state_ += gain * (measurement - observation * state_);

        // Joseph 形式更新 P，在测量方差很小时仍能较好地保持对称和半正定。
        StateCov identity = StateCov::Identity();
        StateCov correction = identity - gain * observation;
        state_cov_ = correction * state_cov_ * correction.transpose()
                   + gain * measurement_noise * gain.transpose();
    }

    const State& state() const { return state_; }

private:
    State state_ = State::Zero();
    StateCov state_cov_ = StateCov::Identity();
};

// 面向 lock 节点的角度 CA（匀角加速度）卡尔曼封装。
//
// 每收到一帧视觉测量，update() 内依次执行：
//   1. 根据相邻图像时间戳计算 dt；
//   2. 用匀角加速度模型预测本帧时刻的角度、角速度和角加速度；
//   3. 将本帧视觉测量用于卡尔曼校正；
//   4. 控制层调用 predict()，再从图像时刻外推到实际控制时刻。
class Kalman_filter_plus
{
public:
    Kalman_filter_plus(const cv::Point2d& angles, const rclcpp::Time& time,
                       const AngleKalmanConfig& config)
        : last_time_(time), config_(config)
    {
        reset(angles);
    }

    void update(const cv::Point2d& input, const rclcpp::Time& time)
    {
        // 第 1 步：使用图像时间戳计算帧间隔，而不是使用函数执行耗时。
        double dt = (time - last_time_).seconds();
        if (!std::isfinite(dt) || dt <= 0.0)
        {
            // 回放、时间戳重复或时钟异常时使用 10 ms 兜底值。
            dt = 0.01;
        }
        // 检测中断时间过长后，旧运动状态已没有参考价值，直接从当前测量重新开始。
        if (dt > 0.1)
        {
            last_time_ = time;
            reset(input);
            return;
        }
        // 防止极小或极大的异常 dt 破坏状态转移和过程噪声矩阵。
        dt = std::clamp(dt, 0.001, 0.1);
        last_time_ = time;

        // 第 2 步：构造 CA 状态转移矩阵。
        // yaw(k) = yaw + yaw_rate*dt + 0.5*yaw_acc*dt^2，pitch 同理。
        StandardKF<6, 2>::TransitionMatrix transition =
            StandardKF<6, 2>::TransitionMatrix::Identity();
        double dt2 = dt * dt;
        transition(0, 1) = dt;
        transition(0, 2) = 0.5 * dt2;
        transition(1, 2) = dt;
        transition(3, 4) = dt;
        transition(3, 5) = 0.5 * dt2;
        transition(4, 5) = dt;

        // 连续白噪声角加加速度模型离散化得到 Q，yaw/pitch 两轴各一组。
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;
        StandardKF<6, 2>::StateCov process_noise =
            StandardKF<6, 2>::StateCov::Zero();
        double q = config_.angular_acceleration_noise;
        set_process_noise_block(process_noise, 0, dt, dt2, dt3, dt4, dt5, q);
        set_process_noise_block(process_noise, 3, dt, dt2, dt3, dt4, dt5, q);

        // 此时状态从上一帧后验值推进到当前图像时刻的先验值。
        filter_.predict(transition, process_noise);

        // 第 3 步：z 是视觉测得的绝对目标角度。
        StandardKF<6, 2>::Measurement z;
        z << input.x, input.y;

        // H 从状态中只取 yaw 和 pitch。
        StandardKF<6, 2>::MeasurementMatrix observation =
            StandardKF<6, 2>::MeasurementMatrix::Zero();
        observation(0, 0) = 1.0;
        observation(1, 3) = 1.0;

        // R 的对角线为 yaw/pitch 测量标准差的平方，两轴暂按互不相关处理。
        StandardKF<6, 2>::MeasurementCov measurement_noise =
            StandardKF<6, 2>::MeasurementCov::Zero();
        measurement_noise(0, 0) = std::pow(config_.measurement_std_yaw_rad, 2);
        measurement_noise(1, 1) = std::pow(config_.measurement_std_pitch_rad, 2);
        filter_.update(z, observation, measurement_noise);
    }

    cv::Point2d predict(double horizon_seconds) const
    {
        // 当前滤波状态位于图像时间戳，继续按估计角速度和角加速度向未来外推。
        // 这里只计算输出，不改变滤波器内部状态，避免影响下一帧测量更新。
        auto state = filter_.state();
        double horizon2 = horizon_seconds * horizon_seconds;
        return {
            state(0) + state(1) * horizon_seconds + 0.5 * state(2) * horizon2,
            state(3) + state(4) * horizon_seconds + 0.5 * state(5) * horizon2
        };
    }

    cv::Point2d angular_velocity() const
    {
        // 返回 yaw_rate 和 pitch_rate，接口与 CV 保持一致。
        auto state = filter_.state();
        return {state(1), state(4)};
    }

private:
    static void set_process_noise_block(StandardKF<6, 2>::StateCov& process_noise,
                                        int offset, double dt, double dt2, double dt3,
                                        double dt4, double dt5, double q)
    {
        process_noise(offset, offset) = dt5 * q / 20.0;
        process_noise(offset, offset + 1) = dt4 * q / 8.0;
        process_noise(offset, offset + 2) = dt3 * q / 6.0;
        process_noise(offset + 1, offset) = dt4 * q / 8.0;
        process_noise(offset + 1, offset + 1) = dt3 * q / 3.0;
        process_noise(offset + 1, offset + 2) = dt2 * q / 2.0;
        process_noise(offset + 2, offset) = dt3 * q / 6.0;
        process_noise(offset + 2, offset + 1) = dt2 * q / 2.0;
        process_noise(offset + 2, offset + 2) = dt * q;
    }

    void reset(const cv::Point2d& angles)
    {
        // 重置时使用当前测量，并恢复速度和加速度的初始协方差。
        double yaw_variance = std::pow(config_.measurement_std_yaw_rad, 2);
        double pitch_variance = std::pow(config_.measurement_std_pitch_rad, 2);
        double velocity_variance = std::pow(config_.initial_velocity_std_rad_s, 2);
        double acceleration_variance = std::pow(config_.initial_acceleration_std_rad_s2, 2);
        filter_.init(angles, yaw_variance, pitch_variance, velocity_variance,
                     acceleration_variance);
    }

    StandardKF<6, 2> filter_;       // 六状态、两测量的线性卡尔曼
    rclcpp::Time last_time_;        // 上一帧图像时间戳，用于计算 dt
    AngleKalmanConfig config_;      // 当前滤波器使用的固定参数
};

}  // namespace tdt_lock
