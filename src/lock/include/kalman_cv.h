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

    // 连续白噪声角加速度强度，对应过程噪声矩阵 Q，单位 rad^2/s^3。
    double angular_acceleration_noise = 0.05;

    // 第一次建立滤波器时角速度的不确定度。值大一些能更快建立目标速度。
    double initial_velocity_std_rad_s = 10.0 * 3.14159265358979323846 / 180.0;

    // 创新门限由基础角度和随帧间隔增长的角度两部分组成。
    double innovation_gate_base_rad = 0.15 * 3.14159265358979323846 / 180.0;
    double innovation_gate_rate_rad_s = 10.0 * 3.14159265358979323846 / 180.0;
};

// 标准线性卡尔曼的最小实现。
// 在本文件中的实际状态为 [yaw, yaw_rate, pitch, pitch_rate]，
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

    void init(const cv::Point2d& angles, double yaw_variance,
              double pitch_variance, double velocity_variance)
    {
        // 初始角度直接使用第一帧测量，初始角速度设为 0。
        state_.setZero();
        state_(0) = angles.x;
        state_(2) = angles.y;

        // P 的对角线表示各状态初始方差。角速度方差较大，表示初速度未知。
        state_cov_.setZero();
        state_cov_(0, 0) = yaw_variance;
        state_cov_(1, 1) = velocity_variance;
        state_cov_(2, 2) = pitch_variance;
        state_cov_(3, 3) = velocity_variance;
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

        // Joseph 形式更新 P。相比 P=(I-KH)P，在角度方差很小时数值稳定性更好。
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

// 面向 lock 节点的角度 CV（匀角速度）卡尔曼封装。
//
// 每收到一帧视觉测量，update() 内依次执行：
//   1. 根据相邻图像时间戳计算 dt；
//   2. 用匀角速度模型预测本帧时刻的角度和角速度；
//   3. 对视觉测量做异常门控；
//   4. 正常测量进入卡尔曼校正，连续三帧异常则重置；
//   5. 控制层调用 predict()，再从图像时刻外推到实际控制时刻。
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
        // 检测中断时间过长后，旧角速度已没有参考价值，直接从当前测量重新开始。
        if (dt > 0.1)
        {
            last_time_ = time;
            reset(input);
            return;
        }
        // 防止极小或极大的异常 dt 破坏状态转移和过程噪声矩阵。
        dt = std::clamp(dt, 0.001, 0.1);
        last_time_ = time;

        // 第 2 步：构造 CV 状态转移矩阵。
        // yaw(k) = yaw(k-1) + yaw_rate(k-1)*dt，pitch 同理。
        StandardKF<4, 2>::TransitionMatrix transition =
            StandardKF<4, 2>::TransitionMatrix::Identity();
        transition(0, 1) = dt;
        transition(2, 3) = dt;

        // 连续白噪声角加速度模型离散化得到 Q：
        // q * [dt^3/3, dt^2/2; dt^2/2, dt]，yaw/pitch 两轴各一组。
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        StandardKF<4, 2>::StateCov process_noise =
            StandardKF<4, 2>::StateCov::Zero();
        double q = config_.angular_acceleration_noise;
        process_noise(0, 0) = dt3 * q / 3.0;
        process_noise(0, 1) = dt2 * q / 2.0;
        process_noise(1, 0) = dt2 * q / 2.0;
        process_noise(1, 1) = dt * q;
        process_noise(2, 2) = dt3 * q / 3.0;
        process_noise(2, 3) = dt2 * q / 2.0;
        process_noise(3, 2) = dt2 * q / 2.0;
        process_noise(3, 3) = dt * q;

        // 此时状态从上一帧后验值推进到当前图像时刻的先验值。
        filter_.predict(transition, process_noise);

        // 第 3 步：使用当前绝对角度测量进行异常门控。
        auto predicted = filter_.state();
        cv::Point2d measurement = input;

        // 门限随 dt 增大，允许目标在低帧率或偶发丢帧时产生更大的正常位移。
        double gate = config_.innovation_gate_base_rad
                    + config_.innovation_gate_rate_rad_s * dt;
        if (std::abs(measurement.x - predicted(0)) > gate
            || std::abs(measurement.y - predicted(2)) > gate)
        {
            ++consecutive_outliers_;
            if (consecutive_outliers_ >= 3)
            {
                // 连续三帧都偏离旧轨迹，认为目标确实跳到了新位置，重新建滤波器。
                reset(measurement);
            }
            // 单帧或双帧异常只使用运动模型预测，不让误检直接拉走状态。
            return;
        }
        consecutive_outliers_ = 0;

        // 第 4 步：z 是视觉测得的绝对目标角度。
        StandardKF<4, 2>::Measurement z;
        z << measurement.x, measurement.y;

        // H 从状态 [yaw, yaw_rate, pitch, pitch_rate] 中只取 yaw 和 pitch。
        StandardKF<4, 2>::MeasurementMatrix observation =
            StandardKF<4, 2>::MeasurementMatrix::Zero();
        observation(0, 0) = 1.0;
        observation(1, 2) = 1.0;

        // R 的对角线为 yaw/pitch 测量标准差的平方，两轴暂按互不相关处理。
        StandardKF<4, 2>::MeasurementCov measurement_noise =
            StandardKF<4, 2>::MeasurementCov::Zero();
        measurement_noise(0, 0) = std::pow(config_.measurement_std_yaw_rad, 2);
        measurement_noise(1, 1) = std::pow(config_.measurement_std_pitch_rad, 2);
        filter_.update(z, observation, measurement_noise);
    }

    cv::Point2d predict(double horizon_seconds) const
    {
        // 第 5 步：当前滤波状态位于图像时间戳，继续按估计角速度向未来外推。
        // horizon = 图像处理延迟 + 串口/云台附加响应延迟。
        // 这里只计算输出，不改变滤波器内部状态，避免影响下一帧测量更新。
        auto state = filter_.state();
        return {
            state(0) + state(1) * horizon_seconds,
            state(2) + state(3) * horizon_seconds
        };
    }

    cv::Point2d angular_velocity() const
    {
        // 返回 yaw_rate 和 pitch_rate，主要用于日志观察与实机调参。
        auto state = filter_.state();
        return {state(1), state(3)};
    }

private:
    void reset(const cv::Point2d& angles)
    {
        // 重置时角度使用当前测量，角速度重新置零，并恢复初始协方差。
        double yaw_variance = std::pow(config_.measurement_std_yaw_rad, 2);
        double pitch_variance = std::pow(config_.measurement_std_pitch_rad, 2);
        double velocity_variance = std::pow(config_.initial_velocity_std_rad_s, 2);
        filter_.init(angles, yaw_variance, pitch_variance, velocity_variance);
        consecutive_outliers_ = 0;
    }

    StandardKF<4, 2> filter_;       // 四状态、两测量的线性卡尔曼
    rclcpp::Time last_time_;        // 上一帧图像时间戳，用于计算 dt
    AngleKalmanConfig config_;      // 当前滤波器使用的固定参数
    int consecutive_outliers_ = 0; // 连续异常测量计数
};

}  // namespace tdt_lock
