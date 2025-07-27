#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <ceres/ceres.h>
#pragma once


constexpr int StateDim = 4; // 状态维度
constexpr int MeasurementDim = 2; // 测量维度
template <int StateDim, int MeasurementDim> class generalEKF {
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


  void init(const cv::Point2f armor_xy) {
      this->state_(0) = armor_xy.x ;
      this->state_(1) = 0;
      this->state_(2) = armor_xy.y;
      this->state_(3) = 0;
    state_cov_.setZero();
    // std::cout<<"state_:"<<state_.transpose()<<std::endl;
  }

  ye_Jacobian calcJacobian_H(
      const Eigen::VectorXd &state,
      const std::function<void(ceres::Jet<double, StateDim> *,
                               const ceres::Jet<double, StateDim> *)> &func) {
    // 初始化状态向量的 Jet 数组
    ceres::Jet<double, StateDim> state_j[StateDim];
    for (int i = 0; i < StateDim; i++) {
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
    for (int i = 0; i < MeasurementDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        jacobian(i, j) = result[i].v[j];
      }
    }
    // 保存测量值
    for (int i = 0; i < MeasurementDim; i++) {
      ye(i) = result[i].a;
    }
    return {ye, jacobian};
  }

  Eigen::MatrixXd calcJacobian_F(const State &state, FunctionType func) {
    // 计算雅可比矩阵
    ceres::Jet<double, StateDim> state_j[StateDim];
    for (int i = 0; i < StateDim; i++) {
      state_j[i] = ceres::Jet<double, StateDim>(state[i], i); // 赋值状态和偏导数
    }

    ceres::Jet<double, StateDim> result[StateDim];

    func(state_j, result);

    // 创建雅可比矩阵
    Eigen::MatrixXd jacobian(StateDim, StateDim);
    for (int i = 0; i < StateDim; i++) {
      for (int j = 0; j < StateDim; j++) {
        jacobian(i, j) = result[i].v[j]; // 将偏导数存入雅可比矩阵
      }
    }

    return jacobian;
  }

  void predict(const FunctionType &func) {
    auto transition_matrix_ = calcJacobian_F(state_, func);
    state_prior_ = transition_matrix_ * state_;
    state_cov_ =
        transition_matrix_ * state_cov_ * transition_matrix_.transpose() +
        process_noise_cov_;
    // std::cout << "先验状态:" << state_ << std::endl;
    // std::cout<<"transition_matrix_:" << transition_matrix_ << std::endl;
  }

  void update(const Measurement &measurement,
              const MeasurementFunctionType &func) {
    auto ye_jacobian = calcJacobian_H(state_prior_, func);
    // std::cout << "ye_jacobian.first:" << ye_jacobian.first << std::endl;
    // std::cout << "ye_jacobian.second:" << ye_jacobian.second << std::endl;
    auto &state_measurement_cov_ = ye_jacobian.second;
    kalman_gain_ = state_cov_ * state_measurement_cov_.transpose() *
                   (state_measurement_cov_ * state_cov_ *
                        state_measurement_cov_.transpose() +
                    measurement_noise_cov_)
                       .inverse();
    // Eigen::MatrixXf S = state_measurement_cov_ * state_cov_ *
    // state_measurement_cov_.transpose() + measurement_noise_cov_;
    // Eigen::LDLT<Eigen::MatrixXf> ldlt(S);  // LDLT 分解、
    // kalman_gain_ = state_cov_ * state_measurement_cov_.transpose() *
    // ldlt.solve(Eigen::MatrixXf::Identity(S.rows(), S.cols())); std::cout <<
    // "kalman_gain:" << kalman_gain_ << std::endl;

    state_ = state_prior_ + kalman_gain_ * (measurement - ye_jacobian.first);

    state_cov_ = (Eigen::Matrix<double, StateDim, StateDim>::Identity() -
                  kalman_gain_ * state_measurement_cov_) *
                 state_cov_;
    // std::cout << "后验状态:" << state_ << std::endl;
  }

  State getState() { return state_; }

  State getPriorState() { return state_prior_; }

  KalmanGain getKalmanGain() { return kalman_gain_; }

  StateCov& getCovariance() { return state_cov_; }
  
  const StateCov& getCovariance() const { return state_cov_; }

  void setCovariance(const StateCov& cov) { state_cov_ = cov; }

  KalmanGain& getKalmangain() { return kalman_gain_; }
  
  const KalmanGain& getKalmangain() const { return kalman_gain_; }

  void setKalmangain(const KalmanGain& kgain) { kalman_gain_ = kgain; }

  void fixState(const State &state) { state_prior_ = state; }

  void set_process_noise_cov(const StateCov &Q) { process_noise_cov_ = Q; }

  void set_measurement_noise_cov(const MeasurementCov &R) { measurement_noise_cov_ = R; }

  void reset() {
    state_.setZero();
    state_cov_.setZero();
    transition_matrix_.setZero();
    process_noise_cov_.setZero();
    measurement_noise_cov_.setZero();
    state_measurement_cov_.setZero();
    kalman_gain_.setZero();
  }


  cv::Point2f getTrackingPos() { // return tracking position
    const double x = state_(0);
    const double y = state_(2);

    return cv::Point2f(x,y);
  }

private:
  static constexpr double INF = 1e9;
  // TODO : 参数变量精简，更名
  State state_;
  State state_prior_;
  StateCov state_cov_;
  StateCov transition_matrix_;
  StateCov process_noise_cov_;
  MeasurementCov measurement_noise_cov_;
  StateMeasurementCov state_measurement_cov_;
  KalmanGain kalman_gain_;
};


class Kalman_filter_plus {
private:
    generalEKF<4,2> KF;
public:

    float Distance(const pcl::PointXY &a, const pcl::PointXY &b) { //计算两点之间的距离
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    double get_time() { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"ms"<< std::endl;
        return duration.count() / 1000.0;
    }

    float last_time = 0;
    std::chrono::steady_clock::time_point timer;//最后更新时间
    float delete_time = 2.0;//超时删除
    std::vector<std::pair<double, pcl::PointXY>> history;
    std::vector<std::pair<int ,int>> detect_history;//第一个是颜色，第二个是数字
    int max_history = 20;

    pcl::PointXY predict_point;
    float detect_r = 1;
    float car_speed = 2;
    float car_max_speed = 3;
    cv::Scalar color;
    bool has_updated = false;
    double dt_=0.1f;

    float r_pos_x= 1;
    float r_pos_y= 1;

    double q_pos_x = 10; // 位置噪声
    double q_pos_v = 100; // 速度噪声


    Kalman_filter_plus(const pcl::PointXY &input,rclcpp::Time time) {  //构造时进行初始化

        cv::Point2f measurement(input.x, input.y); //获取测量值
        predict_point = input;


        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);

        KF.init(measurement);
        
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    int get_color() {
        if(detect_history.size() == 0) {
            return 1;
        }
        //比较队列中的颜色，返回最多的颜色
        int red = 0;
        int blue= 0;
        for(auto &color : detect_history) {
            if(color.first == 0) {
                blue++;
            }
            else {
                red++;
            }
        }
        if(red > blue) {
            return 2;
        }
        else {
            return 0;
        }
    }

    int get_number() {
        int color = get_color();
        std::map<int, int> number_map;
        for(auto &number : detect_history) {
            if(number.first == color) {
                number_map[number.second]++;
            }
        }

        // 初始化最大计数和对应的数字
        int max_count = 0;
        int max_number = 0; // 假设-1为无效数字，或根据实际情况调整
        for(auto &entry : number_map) {
            if(entry.second > max_count) {
                max_count = entry.second;
                max_number = entry.first;
            }
        }
        return max_number; // 返回出现次数最多的数字
    }

    void update(pcl::PointXY &input, rclcpp::Time time) {

        timer = std::chrono::steady_clock::now();

        Eigen::Matrix<double, MeasurementDim, 1> measurement; 
        measurement << input.x, input.y;

        Eigen::Matrix<double, 2, 2> measurement_noise =Eigen::Matrix<double, 2, 2>::Zero();
      
        const std::vector<double> measure_noise_values = {
            r_pos_x,r_pos_y}; //测量噪声

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) {
            for (size_t i = 0; i < diag_values.size(); ++i) {
            matrix(i, i) = diag_values[i];
            }
        };
        set_diagonal(measurement_noise, measure_noise_values);

        KF.set_measurement_noise_cov(measurement_noise);

        auto measureFunction = [](ceres::Jet<double, StateDim> *result,
                            const ceres::Jet<double, StateDim> *state_j) {
            result[0] = state_j[0],
            result[1] = state_j[2];
        };
        KF.update(measurement, measureFunction); // KF update
        auto state = KF.getState();

        predict_point.x = state(0);  
        predict_point.y = state(2); //更新预测点
        has_updated = true;
        last_time = 0;
        auto temp_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), temp_point));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }
    }

    void update_predict_point() { // KF predict
        // std::cout<<"update predict point"<<std::endl;

        dt_ = get_time(); // set dt
        
        double delta_t = dt_; 

        Eigen::Matrix<double, StateDim, StateDim> process_noise = Eigen::Matrix<double, StateDim, StateDim>::Zero(); //set process noise
    

        const std::vector<double> noise_values = { q_pos_x * dt_, q_pos_v * dt_, q_pos_x * dt_, q_pos_v * dt_};

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) {
            for (size_t i = 0; i < diag_values.size(); ++i) {
            matrix(i, i) = diag_values[i];
            }
        };

        set_diagonal(process_noise, noise_values);

        KF.set_process_noise_cov(process_noise);

        // std::cout<<"process_noise:"<<process_noise<<std::endl;

        auto predictFunction = [delta_t](const ceres::Jet<double, StateDim> state_j[StateDim],
           ceres::Jet<double, StateDim> result[StateDim]) {
            result[0] = state_j[0] + state_j[1] * delta_t;
            result[1] = state_j[1];
            result[2] = state_j[2] + state_j[3] * delta_t;
            result[3] = state_j[3];
        };

        KF.predict(predictFunction);
        // std::cout<<"last_time:"<<last_time<<std::endl;
        
        auto result = KF.getPriorState();

        last_time += dt_;
        predict_point.x = result(0);
        predict_point.y = result(2);
    }

    bool match(const pcl::PointXY &input) {
        if(Distance(predict_point, input) < car_max_speed * dt_+detect_r){
            return true;
        } else {
            return false;
        }
    }

    void camera_match(rclcpp::Time &time, pcl::PointXY &input,int color,int number) {
        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        pcl::PointXY match_point;
        for(auto &point : history) {
            // std::cout<<"compare"<<point.first<<"and"<<input_time<<std::endl;
            auto differ = abs(point.first - input_time);
            // std::cout<<"differ"<<differ<<std::endl;
            if(differ < differ_time) {
                differ_time = differ;
                match_point = point.second;
            }
        }
        if (differ_time>TIME_THRESHOLD) {
            return ;
        }//首先找到离相机取帧时间戳最近的点

        
        if(Distance(match_point, input) < detect_r){
            // std::cout<<"match success"<<std::endl;
            detect_history.push_back(std::make_pair(color, number));
            if (detect_history.size() > max_history) {
                detect_history.erase(detect_history.begin());
            }
        }//如果距离小于检测半径，认为匹配成功
    }
    static double GetTimeByRosTime(
        rclcpp::Time& ros_time) {
        double ros_time_value =ros_time.nanoseconds()/1e9;
        // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
        return ros_time_value;
    }
}; 
