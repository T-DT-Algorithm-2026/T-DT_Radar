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
#include <ceres/ceres.h>
#pragma once

namespace tdt_lock 
{
constexpr int StateDim = 6; // 状态维度
constexpr int MeasurementDim = 3; // 测量维度
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


  void init(const cv::Point3f armor_xy) 
  {
      this->state_(0) = armor_xy.x ;
      this->state_(1) = 0;
      this->state_(2) = armor_xy.y;
      this->state_(3) = 0;
      this->state_(4) = armor_xy.z;
      this->state_(5) = 0;
      state_cov_.setZero();//初始化协方差矩阵为0
    // std::cout<<"state_:"<<state_.transpose()<<std::endl;
  }//把装甲板中心的像素坐标 armor_xy 初始化到滤波器的状态向量state_中，并把速度设为 0，同时把协方差矩阵清零。


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
  }//？难道不和上一个函数重复么？


  void predict(const FunctionType &func) 
  {
    auto transition_matrix_ = calcJacobian_F(state_, func);//状态转移矩阵
    state_prior_ = transition_matrix_ * state_;//求Xkhat先验
    state_cov_ = transition_matrix_ * state_cov_ * transition_matrix_.transpose() +process_noise_cov_;//求Pk先验
    // std::cout << "先验状态:" << state_ << std::endl;
    // std::cout<<"transition_matrix_:" << transition_matrix_ << std::endl;
  }//预测的两个矩阵


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
                       .inverse();//计算Kk
    // Eigen::MatrixXf S = state_measurement_cov_ * state_cov_ *
    // state_measurement_cov_.transpose() + measurement_noise_cov_;
    // Eigen::LDLT<Eigen::MatrixXf> ldlt(S);  // LDLT 分解、
    // kalman_gain_ = state_cov_ * state_measurement_cov_.transpose() *
    // ldlt.solve(Eigen::MatrixXf::Identity(S.rows(), S.cols())); std::cout <<
    // "kalman_gain:" << kalman_gain_ << std::endl;

    state_ = state_prior_ + kalman_gain_ * (measurement - ye_jacobian.first);//计算Xkhat后验

    state_cov_ = (Eigen::Matrix<double, StateDim, StateDim>::Identity() -
                  kalman_gain_ * state_measurement_cov_) *
                 state_cov_;//更新Pk后验
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


class Kalman_filter_plus 
{
private:
    generalEKF<6,3> KF;
public:

    float Distance(const pcl::PointXYZ &a, const pcl::PointXYZ &b) 
    { //计算两点之间的距离
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

    double get_time() 
    { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"ms"<< std::endl;
        double t = duration.count() / 1000.0;
        if(t<0.00001)
        {
            t= 0.00001;
        }
        // if(t > 0.3) return 0.1;
        return t;
    }

    float catch_last_time = 0;
    float miss_last_time = 0;
    std::chrono::steady_clock::time_point timer;//最后更新时间
    float delete_time = 2.0;//超时删除
    std::vector<std::pair<double, pcl::PointXYZ>> history;//1.时间 2.点坐标
    int max_history = 20;

    pcl::PointXYZ predict_point;
    cv::Scalar color;
    bool has_updated = false;
    double dt_=0.1f;

    float r_pos_x= 1;
    float r_pos_y= 1;
    float r_pos_z= 1;

    double q_pos_x = 10; // 位置噪声
    double q_pos_v = 100; // 速度噪声

    bool is_space = false;//判断是否是空的卡尔

    Kalman_filter_plus(const pcl::PointXYZ &input,rclcpp::Time time) 
    {  //构造时进行初始化

        cv::Point3f measurement(input.x, input.y, input.z); //获取测量值
        predict_point = input;


        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();//记录当前时间
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);//生成随即颜色

        KF.init(measurement);//传递坐标
        
        has_updated = true;
    }

    ~Kalman_filter_plus() {}


    void update(pcl::PointXYZ &input, rclcpp::Time time) 
    {

        timer = std::chrono::steady_clock::now();

        Eigen::Matrix<double, MeasurementDim, 1> measurement; 
        measurement << input.x, input.y, input.z;//测量矩阵

        Eigen::Matrix<double, 3, 3> measurement_noise =Eigen::Matrix<double, 3, 3>::Zero();//初始化一个 3x3 的零矩阵
      
        const std::vector<double> measure_noise_values = {
            r_pos_x,r_pos_y, r_pos_z}; //测量噪声

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
            result[2] = state_j[4];
        };//提取x，y值
        KF.update(measurement, measureFunction); // KF update//计算点+更新后验
        auto state = KF.getState();//返回state_

        predict_point.x = state(0);  
        predict_point.y = state(2);
        predict_point.z = state(4);
         //更新预测点
        has_updated = true;
        // last_time = 0;//计时器重置
        auto temp_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), temp_point));
        if (history.size() > max_history) {
            history.erase(history.begin());
        }//历史点维护
    }//更新预测点



    void update_predict_point() 
    { // KF predict
        // std::cout<<"update predict point"<<std::endl;

        dt_ = get_time(); // set dt//两帧之间的时间
        
        double delta_t = dt_; 

        Eigen::Matrix<double, StateDim, StateDim> process_noise = Eigen::Matrix<double, StateDim, StateDim>::Zero(); //set process noise
    
        const std::vector<double> noise_values = { q_pos_x * dt_, q_pos_v * dt_, q_pos_x * dt_, q_pos_v * dt_, q_pos_x * dt_, q_pos_v * dt_};//x=10,v=100

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) 
        {
            for (size_t i = 0; i < diag_values.size(); ++i) 
            {
                matrix(i, i) = diag_values[i];
            }
        };//赋值矩阵的对角线

        set_diagonal(process_noise, noise_values);//将噪声值赋值给噪声矩阵4x4

        KF.set_process_noise_cov(process_noise);//传递到process_noise_cov_

        // std::cout<<"process_noise:"<<process_noise<<std::endl;

        auto predictFunction = [delta_t](const ceres::Jet<double, StateDim> state_j[StateDim],
           ceres::Jet<double, StateDim> result[StateDim]) 
        {
            result[0] = state_j[0] + state_j[1] * delta_t;
            result[1] = state_j[1];
            result[2] = state_j[2] + state_j[3] * delta_t;
            result[3] = state_j[3];
            result[4] = state_j[4] + state_j[5] * delta_t; // z
            result[5] = state_j[5];
        };//计算最终预测值

        KF.predict(predictFunction);//先验
        // std::cout<<"last_time:"<<last_time<<std::endl;
        
        auto result = KF.getPriorState();//返回state_prior_

        // last_time += dt_;
        predict_point.x = result(0);
        predict_point.y = result(2);
        predict_point.z = result(4);
    }//先验？

    
    
    cv::Point3f get_predict_point()
    {
        auto state = KF.getState();
        float dug_timr = 0.1;
        float x = state(0)+state(1)*dug_timr;
        float y = state(2)+state(3)*dug_timr;
        float z = state(4)+state(5)*dug_timr;
        return cv::Point3f(x,y,z);

    }


    static double GetTimeByRosTime(rclcpp::Time& ros_time) 
    {
        double ros_time_value =ros_time.nanoseconds()/1e9;
        // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
        return ros_time_value;
    }//将ros时间转换为秒

    void deal_missing(rclcpp::Time time)
    {
        update(predict_point,time);
        miss_last_time+=dt_;
        // catch_last_time-=dt_;
    }//对未识别到的点进行处理

    void deal_catch(pcl::PointXYZ &input, rclcpp::Time time)
    {
        update(input,time);
        miss_last_time=0;//是直接归0还是减@
        is_space=false;
        // if(catch_last_time<1.5)
        // {
        //     catch_last_time+=dt_;
        // }
    }//识别到的点处理
    
}; 

} // namespace tdt_lock