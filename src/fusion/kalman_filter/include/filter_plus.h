#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>

#include <chrono>
#include <array>
#include <cmath>
#include <functional>
#include <utility>
#include <vector>

constexpr int StateDim = 4; // 状态维度
constexpr int MeasurementDim = 2; // 测量维度
template <int StateDim, int MeasurementDim> class generalEKF 
{
  // 通用扩展卡尔曼滤波器
public:
  using FunctionType = std::function<void(const ceres::Jet<double, StateDim>[],
                                          ceres::Jet<double, StateDim>[])>;
  using MeasurementFunctionType =
      std::function<void(ceres::Jet<double, StateDim> *result,
                         const ceres::Jet<double, StateDim> *state_j)>;
  using State = Eigen::Matrix<double, StateDim, 1>;
  using Measurement = Eigen::Matrix<double, MeasurementDim, 1>;
  using StateCov = Eigen::Matrix<double, StateDim, StateDim>;
  using MeasurementCov = Eigen::Matrix<double, MeasurementDim, MeasurementDim>;
  using StateMeasurementCov = Eigen::Matrix<double, MeasurementDim, StateDim>;
  using KalmanGain = Eigen::Matrix<double, StateDim, MeasurementDim>;
  using ye_Jacobian = std::pair<Measurement, StateMeasurementCov>;


  void init(const cv::Point2f armor_xy, const StateCov &initial_cov)
  {
      state_.setZero();
      this->state_(0) = armor_xy.x;
      this->state_(1) = 0;
      this->state_(2) = armor_xy.y;
      this->state_(3) = 0;
      state_prior_ = state_;
      state_cov_ = initial_cov;
  }//把装甲板中心坐标初始化到状态向量中，并设置初始状态协方差。


  ye_Jacobian calcJacobian_H(const Eigen::VectorXd &state, const std::function<void(ceres::Jet<double, StateDim> *, const ceres::Jet<double, StateDim> *)> &func) 
  {
    // 初始化状态向量的 Jet 数组
    ceres::Jet<double, StateDim> state_j[StateDim];
    for (int i = 0; i < StateDim; i++) 
    {
      state_j[i] = ceres::Jet<double, StateDim>(state[i], i);
    }

    // 初始化结果数组（使用 StateDim 维度）
    ceres::Jet<double, StateDim>
        result[StateDim]; // 注意这里是 MeasurementDim = 4

    // 调用测量函数
    func(result, state_j); // 注意参数顺序：result 在前，state_j 在后

    // 创建雅可比矩阵 (4x11)
    Eigen::MatrixXd jacobian(MeasurementDim, StateDim);
    Eigen::MatrixXd ye(MeasurementDim, 1);

    // 填充雅可比矩阵
    for (int i = 0; i < MeasurementDim; i++) 
    {
      for (int j = 0; j < StateDim; j++) 
      {
        jacobian(i, j) = result[i].v[j];
      }
    }
    // 保存测量值
    for (int i = 0; i < MeasurementDim; i++) 
    {
      ye(i) = result[i].a;
    }
    return {ye, jacobian};
  }//计算测量值的雅可比矩阵


  Eigen::MatrixXd calcJacobian_F(const State &state, FunctionType func) 
  {
    // 计算雅可比矩阵
    ceres::Jet<double, StateDim> state_j[StateDim];
    for (int i = 0; i < StateDim; i++) 
    {
      state_j[i] = ceres::Jet<double, StateDim>(state[i], i); // 赋值状态和偏导数
    }

    ceres::Jet<double, StateDim> result[StateDim];

    func(state_j, result);

    // 创建雅可比矩阵
    Eigen::MatrixXd jacobian(StateDim, StateDim);
    for (int i = 0; i < StateDim; i++) 
    {
      for (int j = 0; j < StateDim; j++) 
      {
        jacobian(i, j) = result[i].v[j]; // 将偏导数存入雅可比矩阵
      }
    }

    return jacobian;
  }//计算状态转移函数的雅可比矩阵


  void predict(const FunctionType &func) 
  {
    auto transition_matrix_ = calcJacobian_F(state_, func);//状态转移矩阵
    state_prior_ = transition_matrix_ * state_;//求Xkhat先验
    state_cov_ =
        transition_matrix_ * state_cov_ * transition_matrix_.transpose() +
        process_noise_cov_;//求Pk先验
  }//预测的两个矩阵


  void update(const Measurement &measurement,
              const MeasurementFunctionType &func) {
    auto ye_jacobian = calcJacobian_H(state_prior_, func);
    auto &state_measurement_cov_ = ye_jacobian.second;
    kalman_gain_ = state_cov_ * state_measurement_cov_.transpose() *
                   (state_measurement_cov_ * state_cov_ *
                        state_measurement_cov_.transpose() +
                    measurement_noise_cov_)
                       .inverse();//计算Kk

    state_ = state_prior_ + kalman_gain_ * (measurement - ye_jacobian.first);//计算Xkhat后验

    state_cov_ = (Eigen::Matrix<double, StateDim, StateDim>::Identity() -
                  kalman_gain_ * state_measurement_cov_) *
                 state_cov_;//更新Pk后验
  }

  State getState() const { return state_; }

  State getPriorState() const { return state_prior_; }

  void commit_prediction() { state_ = state_prior_; }

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
    generalEKF<4,2> KF;
public:

    float Distance(const pcl::PointXY &a, const pcl::PointXY &b) 
    { //计算两点之间的距离
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    double get_time() const
    { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        double t = duration.count() / 1000.0;
        return t;
    }

    static double GetTimeByRosTime(rclcpp::Time& ros_time) 
    {
        double ros_time_value =ros_time.nanoseconds()/1e9;
        return ros_time_value;
    }//将ros时间转换为秒

    float miss_last_time = 0;
    std::chrono::steady_clock::time_point timer;//最后更新时间
    double last_radar_catch_time = 0;//最后一次激光雷达点更新时间
    std::vector<std::pair<double, pcl::PointXY>> history;//1.时间 2.点坐标
    std::vector<int> id_history;
    int max_history = 20;

    pcl::PointXY predict_point;
    double last_measurement_time = 0;
    double state_time = 0;
    bool has_measurement = false;
    float detect_r = 0.9; //检测半径
    float car_max_speed = 3;
    bool has_updated = false;
    double dt_=0.1f;

    float r_pos_x= 1;
    float r_pos_y= 1;

    double q_pos_x = 10; // 位置噪声
    double q_pos_v = 100; // 速度噪声

    double p_pos_x = 1; // 初始位置方差
    double p_pos_v = 9; // 初始速度方差，标准差对应最大车速 3 m/s

    int target_id = -1;

    Kalman_filter_plus(const pcl::PointXY &input,rclcpp::Time time) 
    {  //构造时进行初始化

        cv::Point2f measurement(input.x, input.y); //获取测量值
        predict_point = input;
        last_measurement_time = GetTimeByRosTime(time);
        state_time = last_measurement_time;
        has_measurement = true;


        history.push_back(std::make_pair(last_measurement_time, input));
        timer = std::chrono::steady_clock::now();//记录当前时间
        last_radar_catch_time = last_measurement_time;

        Eigen::Matrix<double, StateDim, StateDim> initial_cov = Eigen::Matrix<double, StateDim, StateDim>::Zero();
        initial_cov.diagonal() << p_pos_x, p_pos_v, p_pos_x, p_pos_v;
        KF.init(measurement, initial_cov);//传递初始状态和协方差
        
        has_updated = true;
    }

    int get_target_id() const
    {
        if(id_history.empty()) {
            return -1;
        }

        std::array<int, 12> id_counts{};
        for(const int id : id_history) 
        {
            if(id >= 0 && id < 12) 
            {
                ++id_counts[id];
            }
        }
        int max_count = 0;
        int most_frequent_id = -1;
        for(int id = 0; id < 12; ++id) 
        {
            if(id_counts[id] > max_count) 
            {
                max_count = id_counts[id];
                most_frequent_id = id;
            }
        }
        return most_frequent_id;
    }



    void update(pcl::PointXY &input, rclcpp::Time time) 
    {

        timer = std::chrono::steady_clock::now();

        Eigen::Matrix<double, MeasurementDim, 1> measurement; 
        measurement << input.x, input.y;//测量矩阵

        Eigen::Matrix<double, 2, 2> measurement_noise =Eigen::Matrix<double, 2, 2>::Zero();//初始化一个 2x2 的零矩阵
      
        const std::vector<double> measure_noise_values = {
            r_pos_x,r_pos_y}; //测量噪声

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) {
            for (size_t i = 0; i < diag_values.size(); ++i) {
            matrix(i, i) = diag_values[i];
            }
        };
        set_diagonal(measurement_noise, measure_noise_values);//将噪声值赋值给噪声矩阵

        KF.set_measurement_noise_cov(measurement_noise);//使用设置的 measurement_noise 更新卡尔曼滤波器 KF 的测量噪声协方差矩阵。

        auto measureFunction = [](ceres::Jet<double, StateDim> *result,
                            const ceres::Jet<double, StateDim> *state_j) {
            result[0] = state_j[0],
            result[1] = state_j[2];
        };//提取x，y值
        KF.update(measurement, measureFunction); // KF update//计算点+更新后验
        auto state = KF.getState();//返回state_

        predict_point.x = state(0);  
        predict_point.y = state(2); //更新预测点
        has_updated = true;
        auto temp_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), temp_point));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }//历史点维护
    }//更新预测点



    void update_predict_point() 
    { // KF predict
        dt_ = get_time(); // set dt//两帧之间的时间
        
        double delta_t = dt_; 

        Eigen::Matrix<double, StateDim, StateDim> process_noise = Eigen::Matrix<double, StateDim, StateDim>::Zero(); //set process noise
    
        const std::vector<double> noise_values = { q_pos_x * dt_, q_pos_v * dt_, q_pos_x * dt_, q_pos_v * dt_};//x=10,v=100

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) 
        {
            for (size_t i = 0; i < diag_values.size(); ++i) 
            {
            matrix(i, i) = diag_values[i];
            }
        };//赋值矩阵的对角线

        set_diagonal(process_noise, noise_values);//将噪声值赋值给噪声矩阵4x4

        KF.set_process_noise_cov(process_noise);//传递到process_noise_cov_

        auto predictFunction = [delta_t](const ceres::Jet<double, StateDim> state_j[StateDim],
           ceres::Jet<double, StateDim> result[StateDim]) 
        {
            result[0] = state_j[0] + state_j[1] * delta_t;
            result[1] = state_j[1];
            result[2] = state_j[2] + state_j[3] * delta_t;
            result[3] = state_j[3];
        };//计算最终预测值

        KF.predict(predictFunction);//先验
        
        auto result = KF.getPriorState();//返回state_prior_

        predict_point.x = result(0);
        predict_point.y = result(2);
    }//先验？

    
    bool match(const pcl::PointXY &input) 
    {
        if(Distance(predict_point, input) < car_max_speed * dt_+detect_r)
        {
            return true;
        } 
        else 
        {
            return false;
        }
    }

    bool radar_match(rclcpp::Time &time, pcl::PointXY &input, int detected_target_id)
    {
        if(detected_target_id < 0 || detected_target_id >= 12) {
            return false;
        }

        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        pcl::PointXY match_point;
        for(auto &point : history) 
        {
            auto differ = abs(point.first - input_time);
            if(differ < differ_time) 
            {
                differ_time = differ;
                match_point = point.second;
            }
        }
        if (differ_time>TIME_THRESHOLD) {
            return false;
        }//首先找到离相机取帧时间戳最近的点

        
        if(Distance(match_point, input) < detect_r)
        {
            id_history.push_back(detected_target_id);
            if (id_history.size() > max_history) 
            {
                id_history.erase(id_history.begin());
            }
            target_id = get_target_id();
            return true;
        }
        return false;
    }//雷达的历史点与相机点进行匹配，匹配成功后对统一 ID 投票

    void camera_catch(rclcpp::Time &time, pcl::PointXY &input, int detected_target_id)
    {
        radar_match(time, input, detected_target_id);
    }

    void update_radio(rclcpp::Time &time, pcl::PointXY &input)
    {
        update_predict_point();
        deal_catch(input, time);
    }

    void get_motion_state(pcl::PointXY &position, pcl::PointXY &velocity) const
    {
        const auto state = KF.getState();
        position = pcl::PointXY{static_cast<float>(state(0)), static_cast<float>(state(2))};
        velocity = pcl::PointXY{static_cast<float>(state(1)), static_cast<float>(state(3))};
    }

    bool should_delete(rclcpp::Time &time) const
    {
        return GetTimeByRosTime(time) - last_radar_catch_time > 1.0;
    }


    void deal_missing(rclcpp::Time time)
    {
        // 没有测量时直接采用先验状态，避免把预测点当作虚假测量。
        KF.commit_prediction();
        timer = std::chrono::steady_clock::now();
        state_time = GetTimeByRosTime(time);
        miss_last_time+=dt_;
    }//对未识别到的点进行处理

    void deal_catch(pcl::PointXY &input, rclcpp::Time time)
    {
        last_measurement_time = GetTimeByRosTime(time);
        has_measurement = true;
        update(input,time);
        state_time = last_measurement_time;
        last_radar_catch_time = last_measurement_time;
        miss_last_time=0;
    }//识别到的点处理
}; 
