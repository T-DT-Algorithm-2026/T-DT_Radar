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


constexpr int StateDim = 6; // 状态维度
constexpr int MeasurementDim = 2; // 测量维度
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
      state_(3) = armor_xy.y;
      
      // 修复隐患：不要把协方差矩阵 P 初始化为 0！
      // 赋予初始的不确定性，让滤波器在第一帧就能快速收敛
      state_cov_.setIdentity(); 
      state_cov_(1, 1) = 100.0; // 初始速度极度不确定
      state_cov_(2, 2) = 100.0; // 初始加速度极度不确定
      state_cov_(4, 4) = 100.0; 
      state_cov_(5, 5) = 100.0; 
  }

  // 纯线性的预测方程：X = F * X, P = F * P * F^T + Q
  void predict(const TransitionMatrix& F) 
  {
      state_prior_ = F * state_;
      state_cov_ = F * state_cov_ * F.transpose() + process_noise_cov_;
  }

  // 纯线性的更新方程
  void update(const Measurement& z, const MeasurementMatrix& H) 
  {
      // 1. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
      Eigen::Matrix<double, MeasurementDim, MeasurementDim> S = H * state_cov_ * H.transpose() + measurement_noise_cov_;
      kalman_gain_ = state_cov_ * H.transpose() * S.inverse();

      // 2. 更新后验状态 X = X_prior + K * (Z - H * X_prior)
      state_ = state_prior_ + kalman_gain_ * (z - H * state_prior_);

      // 3. 更新后验协方差 P = (I - K * H) * P
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
    StandardKF<6,2> KF;
public:

    float Distance(const pcl::PointXY &a, const pcl::PointXY &b) 
    { //计算两点之间的距离
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    double get_time() 
    { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count() <<"ms"<< std::endl;
        float t = duration.count();
        // if(t > 0.3) return 0.1;
        return t;
    }

    float catch_last_time = 0;
    float miss_last_time = 0;
    std::chrono::steady_clock::time_point timer;//最后更新时间
    float delete_time = 2.0;//超时删除
    std::vector<std::pair<double, pcl::PointXY>> history;//1.时间 2.点坐标
    std::vector<std::pair<int ,int>> detect_history;//第一个是颜色，第二个是数字
    int max_history = 20;

    pcl::PointXY predict_point;
    float detect_r = 0.9; //检测半径
    float car_speed = 2;
    float car_max_speed = 3;
    cv::Scalar color;
    bool has_updated = false;
    double dt_=0.2f;//两次运行的时间

    float r_pos_x= 10;
    float r_pos_y= 10;

    // double q_pos_x = 10; // 位置噪声
    // double q_pos_v = 10; // 速度噪声
    // double q_pos_a = 0.01; // 加速度噪声
    double noise_a = 0.01; // 加速度噪声

    bool is_space = false;//判断是否是空的卡尔曼
    int now_number=-1;
    int now_color=-1;
    int new_id=-1;


    // 用于 DEMA 滤波的历史变量
    cv::Point2f ema1_last = cv::Point2f(0, 0);
    cv::Point2f ema2_last = cv::Point2f(0, 0);
    bool is_first_output = true;
    
    // 平滑系数 alpha (范围 0 到 1)
    // alpha 越接近 1 越灵敏（跟随卡尔曼），越接近 0 越平滑（抵抗抖动）
    // 建议从 0.4 或 0.5 开始调
    float dema_alpha = 0.6f;


    Kalman_filter_plus(const pcl::PointXY &input,rclcpp::Time time) 
    {  //构造时进行初始化

        cv::Point2f measurement(input.x, input.y); //获取测量值
        predict_point = input;


        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();//记录当前时间
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);//生成随即颜色

        KF.init(measurement);//传递坐标
        
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    int get_color() 
    {
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
    }//0代表蓝色，2代表红色，1代表没有历史点


    int get_number() 
    {
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
    }//得到数字



    void update(pcl::PointXY &input, rclcpp::Time time) 
    {

        timer = std::chrono::steady_clock::now();

        Eigen::Matrix<double, MeasurementDim, 1> measurement; 
        measurement << input.x, input.y;//测量矩阵

        Eigen::Matrix<double, 2, 2> measurement_noise =Eigen::Matrix<double, 2, 2>::Zero();//初始化一个 2x2 的零矩阵
      
        const std::vector<double> measure_noise_values = {r_pos_x,r_pos_y}; //测量噪声

        auto set_diagonal = [](auto &&matrix, const auto &diag_values) {
            for (size_t i = 0; i < diag_values.size(); ++i) {
            matrix(i, i) = diag_values[i];
            }
        };
        set_diagonal(measurement_noise, measure_noise_values);//将噪声值赋值给噪声矩阵

        KF.set_measurement_noise_cov(measurement_noise);//使用设置的 measurement_noise 更新卡尔曼滤波器 KF 的测量噪声协方差矩阵。

        // 构造标准的线性观测矩阵 H (提取索引 0 的 x 和索引 3 的 y)
        Eigen::Matrix<double, MeasurementDim, StateDim> H = Eigen::Matrix<double, MeasurementDim, StateDim>::Zero();
        H(0, 0) = 1.0;
        H(1, 3) = 1.0;

        KF.update(measurement, H); // 直接传入 H 矩阵// KF update//计算点+更新后验
        auto state = KF.getState();//返回state_

        predict_point.x = state(0);  
        predict_point.y = state(3); //更新预测点
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
    
        // const std::vector<double> noise_values = { q_pos_x * dt_, q_pos_v * dt_, q_pos_a * dt_, q_pos_x * dt_, q_pos_v * dt_, q_pos_a * dt_};//x=10,v=100

        // auto set_diagonal = [](auto &&matrix, const auto &diag_values) 
        // {
        //     for (size_t i = 0; i < diag_values.size(); ++i) 
        //     {
        //     matrix(i, i) = diag_values[i];
        //     }
        // };//赋值矩阵的对角线

        // ==================== 👇 开始替换的部分 👇 ====================
        // 删除原来的 set_diagonal 逻辑，改用基于物理运动学推导的耦合矩阵

        double dt2 = dt_ * dt_;
        double dt3 = dt2 * dt_;
        double dt4 = dt3 * dt_;

        // --- X 轴的状态协方差块 (x, vx, ax 分别对应索引 0, 1, 2) ---
        process_noise(0, 0) = 0.25 * dt4 * noise_a;
        process_noise(0, 1) = 0.5 * dt3 * noise_a;
        process_noise(0, 2) = 0.5 * dt2 * noise_a;
        
        process_noise(1, 0) = 0.5 * dt3 * noise_a;
        process_noise(1, 1) = dt2 * noise_a;
        process_noise(1, 2) = dt_ * noise_a;
        
        process_noise(2, 0) = 0.5 * dt2 * noise_a;
        process_noise(2, 1) = dt_ * noise_a;
        process_noise(2, 2) = noise_a;

        // --- Y 轴的状态协方差块 (y, vy, ay 分别对应索引 3, 4, 5) ---
        process_noise(3, 3) = 0.25 * dt4 * noise_a;
        process_noise(3, 4) = 0.5 * dt3 * noise_a;
        process_noise(3, 5) = 0.5 * dt2 * noise_a;
        
        process_noise(4, 3) = 0.5 * dt3 * noise_a;
        process_noise(4, 4) = dt2 * noise_a;
        process_noise(4, 5) = dt_ * noise_a;
        
        process_noise(5, 3) = 0.5 * dt2 * noise_a;
        process_noise(5, 4) = dt_ * noise_a;
        process_noise(5, 5) = noise_a;
        // ==================== 👆 替换结束 👆 ====================

        KF.set_process_noise_cov(process_noise);//传递到process_noise_cov_

        // set_diagonal(process_noise, noise_values);//将噪声值赋值给噪声矩阵4x4

        // KF.set_process_noise_cov(process_noise);//传递到process_noise_cov_      

        // std::cout<<"process_noise:"<<process_noise<<std::endl;

        // 构造标准的线性状态转移矩阵 F (匀加速直线运动模型)
        Eigen::Matrix<double, StateDim, StateDim> F = Eigen::Matrix<double, StateDim, StateDim>::Identity();
        // X 轴的积分关系
        F(0, 1) = delta_t; 
        F(0, 2) = 0.5 * delta_t * delta_t;
        F(1, 2) = delta_t;
        // Y 轴的积分关系
        F(3, 4) = delta_t; 
        F(3, 5) = 0.5 * delta_t * delta_t;
        F(4, 5) = delta_t;

        KF.predict(F); // 直接传入 F 矩阵//先验
        // std::cout<<"last_time:"<<last_time<<std::endl;
        
        auto result = KF.getPriorState();//返回state_prior_

        // last_time += dt_;
        predict_point.x = result(0);
        predict_point.y = result(3);
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

    void camera_match(rclcpp::Time &time, pcl::PointXY &input,int color,int number) 
    {
        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        pcl::PointXY match_point;
        for(auto &point : history) 
        {
            // std::cout<<"compare"<<point.first<<"and"<<input_time<<std::endl;
            auto differ = abs(point.first - input_time);
            // std::cout<<"differ"<<differ<<std::endl;
            if(differ < differ_time) 
            {
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
        now_color = get_color();
        now_number = get_number();
    }//雷达的历史点与相机点进行匹配，匹配成功则将颜色和数字存入detect_history


    static double GetTimeByRosTime(rclcpp::Time& ros_time) 
    {
        double ros_time_value =ros_time.nanoseconds()/1e9;
        // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
        return ros_time_value;
    }//将ros时间转换为秒


cv::Point2f get_predict_point()
    {
        auto state = KF.getState();
        // 这是卡尔曼算出来的原始且可能抖动的预测点
        float x = state(0) + state(1) * dt_ + 0.5 * state(2) * dt_ * dt_; // 如果你退回CV模型，这里的索引应该是 state(0) 和 state(1)
        float y = state(3) + state(4) * dt_ + 0.5 * state(5) * dt_ * dt_; // 如果你退回CV模型，这里的索引应该是 state(3) 和 state(4)
        cv::Point2f raw_predict(x, y);

        // 如果是第一次输出，直接初始化
        if (is_first_output) {
            ema1_last = raw_predict;
            ema2_last = raw_predict;
            is_first_output = false;
            return raw_predict;
        }

        // --- 开始 DEMA 滤波 ---
        
        // 1. 第一次 EMA 计算
        cv::Point2f ema1;
        ema1.x = dema_alpha * raw_predict.x + (1.0f - dema_alpha) * ema1_last.x;
        ema1.y = dema_alpha * raw_predict.y + (1.0f - dema_alpha) * ema1_last.y;
        
        // 2. 第二次 EMA 计算
        cv::Point2f ema2;
        ema2.x = dema_alpha * ema1.x + (1.0f - dema_alpha) * ema2_last.x;
        ema2.y = dema_alpha * ema1.y + (1.0f - dema_alpha) * ema2_last.y;
        
        // 3. 计算最终的 DEMA 结果
        cv::Point2f final_predict;
        final_predict.x = 2.0f * ema1.x - ema2.x;
        final_predict.y = 2.0f * ema1.y - ema2.y;

        // 4. 更新历史值，供下一帧使用
        ema1_last = ema1;
        ema2_last = ema2;

        std::cout << "Time:" << dt_ << "s" << std::endl;
        
        // 返回平滑且无延迟的最终点
        return final_predict;
    }

    
}; 
