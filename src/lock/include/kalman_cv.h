#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <queue>   
#include <memory>   
#include <rclcpp/rclcpp.hpp>
#pragma once

constexpr int StateDim = 4; // x, vx, y, vy
constexpr int MeasurementDim = 2; // x, y

// 纯线性卡尔曼，去掉了冗余的 ceres 依赖
template <int StateDim, int MeasurementDim> class StandardKF 
{
public:
  using State = Eigen::Matrix<double, StateDim, 1>;
  using Measurement = Eigen::Matrix<double, MeasurementDim, 1>;
  using StateCov = Eigen::Matrix<double, StateDim, StateDim>;
  using MeasurementCov = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
  using TransitionMatrix = Eigen::Matrix<double, StateDim, StateDim>;
  using MeasurementMatrix = Eigen::Matrix<double, MeasurementDim, StateDim>;
  using KalmanGain = Eigen::Matrix<double, StateDim, MeasurementDim>;

  void init(const cv::Point2f armor_xy) 
  {
      state_.setZero();
      state_(0) = armor_xy.x;
      state_(1) = 0; // vx
      state_(2) = armor_xy.y;
      state_(3) = 0; // vy
      
      state_cov_.setIdentity(); 
      state_cov_(0, 0) = 1.0;  // 初始位置方差小
      state_cov_(2, 2) = 1.0; 
      state_cov_(1, 1) = 1000.0; // 初始速度方差极大
      state_cov_(3, 3) = 1000.0; 
  }

  void predict(const TransitionMatrix& F) 
  {
      state_prior_ = F * state_;
      state_cov_ = F * state_cov_ * F.transpose() + process_noise_cov_;
  }

  void update(const Measurement& z, const MeasurementMatrix& H) 
  {
      Eigen::Matrix<double, MeasurementDim, MeasurementDim> S = H * state_cov_ * H.transpose() + measurement_noise_cov_;
      kalman_gain_ = state_cov_ * H.transpose() * S.inverse();
      state_ = state_prior_ + kalman_gain_ * (z - H * state_prior_);
      Eigen::Matrix<double, StateDim, StateDim> I = Eigen::Matrix<double, StateDim, StateDim>::Identity();
      state_cov_ = (I - kalman_gain_ * H) * state_cov_;
  }

  State getState() { return state_; }
  State getPriorState() { return state_prior_; }
  void set_process_noise_cov(const StateCov &Q) { process_noise_cov_ = Q; }
  void set_measurement_noise_cov(const MeasurementCov &R) { measurement_noise_cov_ = R; }

private:
  State state_;
  State state_prior_;
  StateCov state_cov_;
  StateCov process_noise_cov_;
  MeasurementCov measurement_noise_cov_;
  KalmanGain kalman_gain_;
};


class Kalman_filter_plus 
{
private:
    StandardKF<4,2> KF; 
public:

    float Distance(const pcl::PointXY &a, const pcl::PointXY &b) { return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)); }

    double get_time() 
    { 
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        return duration.count() / 1000.0; 
    }

    std::chrono::steady_clock::time_point timer;
    std::vector<std::pair<double, pcl::PointXY>> history;
    std::vector<std::pair<int ,int>> detect_history;
    int max_history = 20;

    pcl::PointXY predict_point;
    float detect_r = 0.9; 
    float car_max_speed = 3;
    bool has_updated = false;
    double dt_=0.01;

    float r_pos_x= 1;
    float r_pos_y= 1;
    double noise_v = 20.0; // 速度噪声

    // DEMA 滤波变量
    cv::Point2f ema1_last = cv::Point2f(0, 0);
    cv::Point2f ema2_last = cv::Point2f(0, 0);
    bool is_first_output = true;
    float dema_alpha = 0.8f;


    Kalman_filter_plus(const pcl::PointXY &input, rclcpp::Time time) 
    {  
        cv::Point2f measurement(input.x, input.y);
        predict_point = input;
        timer = std::chrono::steady_clock::now();
        KF.init(measurement);
        has_updated = true;
    }

    void update(pcl::PointXY &input, rclcpp::Time time) 
    {
        timer = std::chrono::steady_clock::now();
        Eigen::Matrix<double, 2, 1> measurement; 
        measurement << input.x, input.y;

        Eigen::Matrix<double, 2, 2> measurement_noise = Eigen::Matrix<double, 2, 2>::Zero();
        measurement_noise(0,0) = r_pos_x;
        measurement_noise(1,1) = r_pos_y;

        KF.set_measurement_noise_cov(measurement_noise);

        // 构建观测矩阵 H [x, y] 分别对应索引 0 和 2
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0, 0) = 1.0;
        H(1, 2) = 1.0;

        KF.update(measurement, H);
        auto state = KF.getState();
        predict_point.x = state(0);  
        predict_point.y = state(2); 
        has_updated = true;
    }

    void update_predict_point() 
    {
        dt_ = get_time(); 
        double delta_t = dt_; 
        // std::cout << "Delta t: " << delta_t << " seconds" << std::endl;

        Eigen::Matrix<double, 4, 4> process_noise = Eigen::Matrix<double, 4, 4>::Zero(); 
        
        // CV 模型的离散白噪声矩阵 Q
        double dt2 = dt_ * dt_;
        double dt3 = dt2 * dt_;
        
        // X 轴 (x, vx)
        process_noise(0, 0) = 0.33 * dt3 * noise_v;
        process_noise(0, 1) = 0.5 * dt2 * noise_v;
        process_noise(1, 0) = 0.5 * dt2 * noise_v;
        process_noise(1, 1) = dt_ * noise_v;

        // Y 轴 (y, vy)
        process_noise(2, 2) = 0.33 * dt3 * noise_v;
        process_noise(2, 3) = 0.5 * dt2 * noise_v;
        process_noise(3, 2) = 0.5 * dt2 * noise_v;
        process_noise(3, 3) = dt_ * noise_v;

        KF.set_process_noise_cov(process_noise);

        // 构建状态转移矩阵 F (CV 模型)
        Eigen::Matrix<double, 4, 4> F = Eigen::Matrix<double, 4, 4>::Identity();
        F(0, 1) = delta_t; // x = x + vx*dt
        F(2, 3) = delta_t; // y = y + vy*dt

        KF.predict(F);
        auto result = KF.getPriorState();
        predict_point.x = result(0);
        predict_point.y = result(2);
    }

    cv::Point2f get_predict_point()
    {
        auto state = KF.getState();
        float x = state(0) + state(1) * dt_; 
        float y = state(2) + state(3) * dt_; 
        cv::Point2f raw_predict(x, y);

        if (is_first_output) {
            ema1_last = raw_predict; 
            ema2_last = raw_predict;
            is_first_output = false;
            return raw_predict;
        }

        cv::Point2f ema1;
        ema1.x = dema_alpha * raw_predict.x + (1.0f - dema_alpha) * ema1_last.x;
        ema1.y = dema_alpha * raw_predict.y + (1.0f - dema_alpha) * ema1_last.y;
        
        cv::Point2f ema2;
        ema2.x = dema_alpha * ema1.x + (1.0f - dema_alpha) * ema2_last.x;
        ema2.y = dema_alpha * ema1.y + (1.0f - dema_alpha) * ema2_last.y;
        
        cv::Point2f final_predict;
        final_predict.x = 2.0f * ema1.x - ema2.x;
        final_predict.y = 2.0f * ema1.y - ema2.y;

        ema1_last = ema1; ema2_last = ema2;
        return final_predict; 
    }

    static double GetTimeByRosTime(rclcpp::Time& ros_time) { return ros_time.nanoseconds()/1e9; }
};