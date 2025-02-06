#ifndef __BASE_TOOLKIT
#define __BASE_TOOLKIT
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <boost/functional/hash.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "base_msg.h"
#include "base_param.h"
namespace tdttoolkit {
struct Polar3f {
  Polar3f(float d, float y, float p) : distance(d), yaw(y), pitch(p) {}
  Polar3f() = default;
  Polar3f(const Polar3f& other)
      : distance(other.distance), yaw(other.yaw), pitch(other.pitch) {}

  float distance = 0;
  float yaw = 0;
  float pitch = 0;

  friend std::ostream& operator<<(std::ostream&, Polar3f&);
};

class Time {
 public:
  /**
   * @name    GetTimeNow
   * @brief   获取从程序开始运行到当下的时间戳
   * @return  时间戳(单位：us)
   */
  static inline uint64_t GetTimeNow() {
    auto cnt_clock = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(cnt_clock -
                                                                 init_clock_)
        .count();
  };

  static inline builtin_interfaces::msg::Time GetROSTimeNow() {
    auto ros_time_value = GetTimeNow() + time_diff_;
    builtin_interfaces::msg::Time ret_time;
    ret_time.set__sec(ros_time_value / 1000000);
    ret_time.set__nanosec((ros_time_value % 1000000) * 1000);
    return ret_time;
  }

  static inline builtin_interfaces::msg::Time GetRosTimeByTime(uint64_t time) {
    builtin_interfaces::msg::Time ret_time;
    ret_time.set__sec((time + time_diff_) / 1000000);
    ret_time.set__nanosec(((time + time_diff_) % 1000000) * 1000);
    // TDT_DEBUG("time: %d, sec: %d, nsec: %d", time, ret_time.sec,
    //           ret_time.nanosec);
    return ret_time;
  }

  static inline uint64_t GetTimeByRosTime(
      builtin_interfaces::msg::Time ros_time) {
    uint64_t ros_time_value = (uint64_t)ros_time.sec * 1000000.0 +
                              (uint64_t)ros_time.nanosec / 1000.0;
    return ros_time_value - time_diff_;
  }

  /**
   * @name    Init
   * @brief   初始化程序时间戳和与ROS时间戳的差值(可选)
   * @param   [in] ros_time 时间戳(单位：us)
   */
  static inline void Init(const builtin_interfaces::msg::Time& ros_time =
                              builtin_interfaces::msg::Time()) {
    frame_time_ = 0;
    init_clock_ = std::chrono::high_resolution_clock::now();
    time_diff_ =
        (uint64_t)ros_time.sec * 1e6 + (uint64_t)ros_time.nanosec / 1e3;
  }

  static inline void UpdateFrameTime(uint64_t tricktime) {
    ++frame_id_;
    frame_time_ = tricktime;
  }

  static inline uint64_t GetFrameId() { return frame_id_; }

  static inline uint64_t GetFrameTime() { return frame_time_; }

  static inline uint64_t GetFrameRunTime() {
    return GetTimeNow() - frame_time_;
  }

 private:
  static uint64_t frame_id_;
  static uint64_t frame_time_;

  static std::chrono::_V2::system_clock::time_point init_clock_;
  static uint64_t time_diff_;
};

/**
 * @name    KalmanFilter 卡尔曼滤波器
 * @brief   重新封装了cv::KalmanFilter, 只适用状态量为[1 x 1]动态模型
 * @note    只适用于状态量维度为1x1的动态模型, 若要预测多维模型, 需要重写
 */
class KalmanFilter : public cv::KalmanFilter {
 public:
  void init(float init);

  /**
   * @name    构造函数
   * @brief   初始化滤波器,
   * @param   time_limit 相邻两次使用Estimate()的时间间隔上限,
   * 超过该上限就判定为时间不连续
   * @param   reset_flag 是否在判定为 "数据不连续"
   * 时将最优估值置零，若为false则会将本次测量值作为最优估值
   */
  KalmanFilter();

  /**
   * @name    SetQ
   * @brief   设置预测过程噪声方差Q(k)为q*q, 对应成员processNoiseCov
   * @note    要预测的量都是1x1的状态量,所以协方差矩阵只有一个元素,也就是方差
   */
  inline void SetQ(float q) {
    setIdentity(this->processNoiseCov, cv::Scalar::all(q * q));
  }

  /**
   * @name    SetProcessNoiseCov
   * @brief   设置预测过程噪声方差R(k)为r*r, 对应成员measurementNoiseCov
   * @note    与Q同理
   */
  inline void SetR(float r) {
    setIdentity(this->measurementNoiseCov, cv::Scalar::all(r * r));
  }

  /**
   * @name    Estimate
   * @brief
   * 整合了cv::KalmanFilter::predict()和correct(),并设置了连续性判定和置零判定
   * @param   measurement 测量值
   * @param   isContinuous 数据是否连续（例如更换了滤波目标就是不连续）
   * @return  statePost 最优估计值
   */
  float Estimate(float measurement);

  /**
   * @brief   从cv::KalmanFilter继承的public属性, 可以直接访问
   * ----------------------最有用的-------------------------
   * @name    gain                卡尔曼增益
   * @name    statePre            先验预测值  ^x(k|k-1)
   * @name    statePost           后验估计值 ^x(k|k)
   * ---------------------可能有用的-------------------------
   * @name    processNoiseCov     先验预测噪声的协方差矩阵 Q(k)
   * @name    measurementNoiseCov 测量噪声的协方差矩阵 R(k)
   * @name    errorCovPre         先验预测的协方差矩阵 P(k|k-1)
   * @name    errorCovPost        后验预测协方差矩阵 P(k|k)
   * --------------------没啥访问必要的-----------------------
   * @name    transitionMatrix    状态转移矩阵 A(k)
   * @name    measurementMatrix   规格转化矩阵 H(k)
   * @name    controlMatrix       控制矩阵 B(k)
   * ------------------------------------------------------
   */
  float GetLastTime() { return last_time_; }

 private:
  float time_limit_;
  float last_time_;
  bool reset_flag_;
};

class EKF : public cv::KalmanFilter {
 public:
  EKF(){};  // 改为了全局变量，于是构造时不进行初始化

  void InitParam(std::string param_name_, int status = 0) {  // 初始化参数
    if (!status) {
      LoadParam::ReadParam(param_name_, "sigma_ax_az", sigma_ax);
      sigma_az = sigma_ax ; 
      LoadParam::ReadParam(param_name_, "sigma_yaw", sigma_yaw);
      LoadParam::ReadParam(param_name_, "sigma_dis", sigma_dis);
      LoadParam::ReadParam(param_name_, "sentryInit", sentryInit);
      LoadParam::ReadParam(param_name_, "normalInit", normalInit);
    } else {
      LoadParam::ReadParam(param_name_, "sigma_ax_spin", sigma_ax);
      LoadParam::ReadParam(param_name_, "sigma_az_spin", sigma_az);
      LoadParam::ReadParam(param_name_, "sigma_yaw_spin", sigma_yaw);
      LoadParam::ReadParam(param_name_, "sigma_dis_spin", sigma_dis);
    }
  };
  void init(cv::Point3f cntPoint = cv::Point3f(-1, -1, -1),
            bool toward = false) {
    if (cntPoint == cv::Point3f(-1, -1, -1)) {
      statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
      errorCovPost = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 100, 0, 0, 0, 0,
                      1, 0, 0, 0, 0, 100);
      statePre = statePost ; 
    } else if (toward) {
      statePost = (cv::Mat_<float>(4, 1) << cntPoint.x,
                   normalInit * statePost.at<float>(1, 0), cntPoint.z,
                   normalInit * statePost.at<float>(3, 0));
      statePre = statePost ; 
    } else {
      statePost = (cv::Mat_<float>(4, 1) << cntPoint.x, 0, cntPoint.z, 0);
      errorCovPost = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 100, 0, 0, 0, 0,
                      1, 0, 0, 0, 0, 100);
       statePre = statePost ; 
    }
  };

  void SetQ(float DeltaT);
  void SetR();
  void Predict();
  void Correct(const cv::Mat& measurement, bool ifContinous = true);
  cv::Mat hx();
  inline cv::Mat getStatePre(){
    return statePre;
  }

  inline cv::Mat getStatePost(){
      return statePost;
  }

 private:
  float sigma_ax;    // x方向加速度噪声
  float sigma_az;    // z方向加速度噪声
  float sigma_yaw;   // yaw测量噪声
  float sigma_dis;   // 水平距离测量噪声
  float sentryInit;  // 初始化后速度比例
  float normalInit;
  cv::Mat measure_err;
  cv::Mat convariance;
  cv::Mat Jacob;
};

/**
 * @brief Differential_Evolution差分进化算法
 * @note 用DE算法求出最优解
 * @param [in]
 * 算法设定参数：种群规模，缩放因子，交叉率，迭代次数，维度和维度上限
 * @param [in] 适应度方程
 * @return 待求变量的最优解
 */
class Differential_Evolution {
  // 参考网址https://www.pianshen.com/article/4619283970/
 public:
  struct Single {                 // 代表个体
    std::vector<double> m_xreal;  // n维坐标，存储飞镖射出时的V和θ
    double m_fitness;             // 适应度
    Single() {                    // 构造函数
      m_xreal = std::vector<double>(5, 0);  // 初始化坐标,最大支持5个变量
      m_fitness = 0;                        // 初始化适应度
    }
  };

  // 在类中初始化vector
  std::vector<struct Single> parentpop =
      std::vector<struct Single>(10);  // 父种群
  std::vector<struct Single> childpop =
      std::vector<struct Single>(10);  // 子种群

  /**
   * @brief 得到range范围内一个随机实数
   * @param 生成随机数的写法包含了对0到1的处理
   * @return 任意范围内的随机数
   */
  double RandReal(double left, double right);

  /**
   * @brief 得到range范围内一个随机整数
   * @return 任意范围内的整数
   */
  double RandInt(int left, int right);

  /**
   * @brief 输出群体情况
   * @note cout加速
   */
  void Print();

  /**
   * @brief 初始化群体，为父群体赋初值
   */
  void InitPop();

  /**
   * @brief 外部定义的适应度函数
   * @note 此函数需要用到DE算法类内的变量
   * @param [in] parentpop的遍历index
   * @return 待求的最优解
   */
  virtual double CalculateFitness(std::vector<double> realx) = 0;

  /**
   * @brief 随机选择三个不相同的父亲个体
   */
  void ParentSelection(std::vector<int>& index);

  /**
   * @brief 变异操作
   * @note
   * 把种群中任意两个个体的向量差乘上变异因子，与第三个个体求和来产生新个体
   * @note 越界处理
   * @param 传入的随机三个父个体
   * @return
   */
  void Mutation(std::vector<int>& index, int i);

  /**
   * @brief 交叉操作
   * @param crossRate，在这里影响收敛速度
   */
  void BinCrossover(int i);

  /**
   * @brief 父代与子代选择
   * @param 在这里添加约束条件决定让种群以怎样的方式收敛
   * @return 将优种赋给父类种群
   */
  void Selection(int i);

  /**
   * @brief 调用的总函数
   */
  std::vector<double> GetDE_Result();

  void setSize(int size_) {
    this->SIZE = size_;
    // 每次更改重新初始化
    parentpop = std::vector<struct Single>(SIZE);
    childpop = std::vector<struct Single>(SIZE);
  }
  inline void setDimension(int dimension_) { this->DIMENSION = dimension_; }
  inline void setF(double f_) { this->F = f_; }
  inline void setCROSSOVER_RATE(double crossover_rate_) {
    this->CROSSOVER_RATE = crossover_rate_;
  }
  inline void setITERNUM(int iter_num_) { this->ITER_NUM = iter_num_; }
  inline void setLOWER_BOUND(std::vector<double> lower_bound_) {
    this->LOWER_BOUND = lower_bound_;
  }
  inline void setUPPER_BOUND(std::vector<double> upper_bound_) {
    this->UPPER_BOUND = upper_bound_;
  }

 private:
  int DIMENSION = 2;  // 目标函数维数(变量个数)
  int SIZE = 10;      // 种群规模
  double F = 0.6;  // 缩放因子,此参数调大会加大变异的程度，使种群多样性增加
  double CROSSOVER_RATE = 0.6;  // 交叉率,此参数调高会加快收敛速度
  int ITER_NUM = 30;            // 迭代次数
  std::vector<double>
      LOWER_BOUND;  // 每一个维度的上限,从前往后按顺序输入设的变量
  std::vector<double> UPPER_BOUND;  // 每一个维度的下限,和上面同理
};

//////////////////////////////////////调用方法演示///////////////////////////////////////
/*
    //定义一个子类继承DE类
    class DE_in_Buff : public tdttoolkit::Differential_Evolution {
        double CalculateFitness(std::vector<double> realx);
    };
    //自定义适应度方程
    double DE_in_Buff::CalculateFitness(std::vector<double> realx) {
        double result = realx[0] * realx[0] + realx[1] * realx[1];
        return result;
    }

    DE_in_Buff DE;

    //调用DE
    void Buff::use() {
        //以下是必须要设置的：
        DE.setDimension(2);
        std::vector<double> lower{15,
   40};//声明变量的最小范围，从左到右依次是要计算的变量 std::vector<double>
   upper{18, 45};//最大范围，同上，记住要算几个变量就设几个数,和维度数保持一致
        DE.setLOWER_BOUND(lower);//设置范围
        DE.setUPPER_BOUND(upper);

        //以下是可设可不设的，根据实际情况决定
        DE.setSize(10);//设置种群大小,太大计算量丰富但耗时长，太小不利于得到全局最优解
        DE.setITERNUM(25);//迭代次数，20到30次就收敛的差不多了，取决于问题的复杂度

        //以下基本上0.5左右就ok了
        DE.setF(0.6);//设置缩放因子，调大会加大变异的程度，使种群多样性增加
        DE.setCROSSOVER_RATE(0.6);//设置交叉率,调大会加快收敛速度，但会让进化变得苛刻

        //调用的总函数
        DE.GetDE_Result();
    }
*/

/*****************
 * @name  PolyFit
 * @brief 计算多项式回归下的系数矩阵
 *
 * @param [in] array_x 自变量的数组
 * @param [in] array_y 因变量的数组
 * @param [in] size    数组大小
 * @param [in] n       拟合的阶数
 * @return 获得的系数矩阵
 ****************/
cv::Mat PolyFit(float array_x[], float array_y[], int size, int n);

/*
 * @brief 贝塞尔多项式拟合 基于de castulo 算法
 * @param 拟合的点 ， 区间值t
 * @author created by:郑乐 2360838113@qq.com  [date]:2022.7.9
 * */
cv::Point2f Bezier(std::vector<cv::Point2f> points, float t);

/**
 * @name    EulerAnglesToRotationMatrix
 * @brief   欧拉角转旋转矩阵, order为true时为Z-Y-X欧拉角, false时为X-Y-Z欧拉角
 * @param   输入的欧拉角方向为 : 顺正逆负
 * @return  Mat 旋转矩阵
 */
cv::Mat EulerAnglesToRotationMatrix(cv::Vec3f& euler, bool order);

/*积分*/
float integral(std::vector<cv::Point2f> speedPoints);

Eigen::MatrixXf regress(std::vector<float> x, std::vector<float> y);

size_t MatHash(const cv::Mat& mat);

template <typename _Tp>
class Vec2 {
 public:
  _Tp x, y;
  Vec2() : x(0), y(0) {}
  Vec2(_Tp x_, _Tp y_) : x(x_), y(y_) {}
  Vec2(const Vec2& v) : x(v.x), y(v.y) {}
  Vec2& operator=(const Vec2& v) {
    x = v.x;
    y = v.y;
    return *this;
  }
  Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
  Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
  Vec2 operator*(const _Tp& v) const { return Vec2(x * v, y * v); }
  Vec2 operator/(const _Tp& v) const { return Vec2(x / v, y / v); }
  Vec2& operator+=(const Vec2& v) {
    x += v.x;
    y += v.y;
    return *this;
  }
  Vec2& operator-=(const Vec2& v) {
    x -= v.x;
    y -= v.y;
    return *this;
  }
  Vec2& operator*=(const _Tp& v) {
    x *= v;
    y *= v;
    return *this;
  }
  Vec2& operator/=(const _Tp& v) {
    x /= v;
    y /= v;
    return *this;
  }
  // 点积
  _Tp dot(const Vec2& v) const { return x * v.x + y * v.y; }
  // 叉积
  _Tp cross(const Vec2& v) const { return x * v.y - y * v.x; }
  // 长度
  _Tp length() const { return sqrt(x * x + y * y); }
  // 长度的平方
  _Tp length2() const { return x * x + y * y; }
  // 归一化
  Vec2& normalize() {
    _Tp l = length();
    x /= l;
    y /= l;
    return *this;
  }
  // 返回归一化后的向量
  Vec2 normalized() const {
    _Tp l = length();
    return Vec2(x / l, y / l);
  }
  // 逆时针旋转
  Vec2& rotate(_Tp angle) {
    _Tp c = cos(angle);
    _Tp s = sin(angle);
    _Tp nx = x * c - y * s;
    _Tp ny = x * s + y * c;
    x = nx;
    y = ny;
    return *this;
  }
  // 返回逆时针旋转后的向量
  Vec2 rotated(_Tp angle) const {
    _Tp c = cos(angle);
    _Tp s = sin(angle);
    return Vec2(x * c - y * s, x * s + y * c);
  }

  static Vec2 polor(double r, double theta) {
    return Vec2(r * cos(theta), r * sin(theta));
  }

  static Vec2 polor(double theta) { return Vec2(cos(theta), sin(theta)); }

  double angle() const { return atan2(y, x); }

  double angle(const Vec2& v) const {
    double cos_theta = dot(v) / (length() * v.length());
    return acos(cos_theta);
  }

  double angle(const Vec2& v1, const Vec2& v2) const {
    double cos_theta = (v1 - *this).dot(v2 - *this) /
                       ((v1 - *this).length() * (v2 - *this).length());
    return acos(cos_theta);
  }

  double size() const { return sqrt(x * x + y * y); }
};

typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;
typedef Vec2<int> Vec2i;

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);

class PID {
 public:
  PID(){};
  void init(double Kp, double Ki, double Kd, double dt, double max, double min,
            double imax);
  double calculate(double setpoint, double pv);

 private:
  double Kp, Ki, Kd;
  double dt;
  double max, min;
  double imax;
  double pre_error;
  double integral;
};

struct ShootPlatform {
  float pitch = 0;
  float yaw = 0;
  float bulletspeed = 2.7f;
  double timeStamp = 0;
};

bool Rodrigues(cv::InputArray _src, cv::OutputArray _dst,
               cv::OutputArray _jacobian = cv::noArray());

/**
 * @name    RectangularToPolar
 * @param   Rectangular 世界系中的xyz直角坐标点
 * @return  Polar3f (distance, pitch, yaw) 极坐标点
 */
Polar3f RectangularToPolar(cv::Point3f const& rectangular_point);
void AngleCorrect(float &angle);

/**
 * @name    PolarToRectangular
 * @param   polar_point 世界系中的Polar极坐标点
 * @return  Point3f 世界系中的xyz直角坐标点
 */
cv::Point3f PolarToRectangular(const Polar3f& polar_point);

float CalcAngleDifference(float a, float b);

enum ArmorType { Empty = 0, Lightbar = 1, NoLightbar = 2 };

enum RobotType {
  TYPEUNKNOW = 0,
  HERO = 1,
  ENGINEER = 2,
  INFANTRY3 = 3,
  INFANTRY4 = 4,
  INFANTRY5 = 5,
  OUTPOST = 6,
  SENTRY = 7,
  BASE = 8
};
enum DPRobotType {
  No = 0,
  BLUE_1 = 1,
  BLUE_2 = 2,
  BLUE_3 = 3,
  BLUE_4 = 4,
  BLUE_5 = 5,
  BLUE_6 = 6,
  BLUE_7 = 7,
  BLUE_8 = 8,
  RED_1 = 9,
  RED_2 = 10,
  RED_3 = 11,
  RED_4 = 12,
  RED_5 = 13,
  RED_6 = 14,
  RED_7 = 15,
  RED_8 = 16,
  ROBOT = 17
};

enum BuffType { None = 0, FlowWater = 1, NoFlowWater = 2, R = 3 };

enum Mode { ArmorMode = 0, EnergyBuffMode = 1, BuffDisturb = 2 };

/**
 * @name    WorldPointsList
 * @brief  用于PNP解算的三维点集
 */
struct WorldPointLists {
  static inline void GetRobotWorldPoints(RobotType robot_type,
                                         std::vector<cv::Point3f>& points) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    points = robot_world_points_list[int(robot_type)];
  }
  static inline void GetDPRobotWorldPoints(DPRobotType DP_robot_type,
                                           std::vector<cv::Point3f>& points,
                                           int balance[3]) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    if ((DP_robot_type == 3 && balance[0] == 2) ||
        (DP_robot_type == 4 && balance[1] == 2) ||
        (DP_robot_type == 5 && balance[2] == 2) ||
        (DP_robot_type == 11 && balance[0] == 2) ||
        (DP_robot_type == 12 && balance[1] == 2) ||
        (DP_robot_type == 13 && balance[2] == 2)) {
      points = robot_world_points_list[9];
    } else {
      if (DP_robot_type < 9) {
        points = robot_world_points_list[int(DP_robot_type)];
      }
      if (DP_robot_type > 8) {
        points = robot_world_points_list[int(DP_robot_type) - 8];
      }
    }
  }

  static inline void GetDPRobotWorldPoints(DPRobotType DP_robot_type,
                                           std::vector<cv::Point3f>& points,
                                           bool isBalance) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    if (isBalance) {
      points = robot_world_points_list[9];
    } else {
      if (DP_robot_type < 9) {
        points = robot_world_points_list[int(DP_robot_type)];
      }
      if (DP_robot_type > 8) {
        points = robot_world_points_list[int(DP_robot_type) - 8];
      }
    }
  }
  static inline void GetRobotWorldPoints(RobotType robot_type,
                                         std::vector<cv::Point3f>& points,
                                         int balance[3]) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    if ((int(robot_type) == 3 && balance[0] == 2) ||
        (int(robot_type) == 4 && balance[1] == 2) ||
        (int(robot_type) == 5 && balance[2] == 2)) {
      points = robot_world_points_list[9];
    } else {
      points = robot_world_points_list[int(robot_type)];
    }
  }

  static inline void GetBuffWorldPoints(int idex_,
                                        std::vector<cv::Point3f>& points) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    points = buff_world_points_list[idex_];
  }

  static inline void GetBuffDisturbWorldPoints(
      std::vector<cv::Point3f>& points) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    points = buff_disturb_world_points_list;
  }

  static void PushBuffWorldPoints(int idex_, std::vector<cv::Point3f>& points) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    std::vector<cv::Point3f> tmppoints;
    points.insert(points.end(), buff_world_points_list[idex_].begin(),
                  buff_world_points_list[idex_].end());
  }

  static cv::Point3f GetBuffWorldPoints_Index(int i, int j) {
    if (is_first_time_) {
      CalcPoints();
      is_first_time_ = false;
    }
    return buff_world_points_list[i][j];
  }

  static inline float GetLightHeight() { return kLightBarHeight; }

  static inline float GetArmorWidth(int type, bool isBalance) {
    if (isBalance)  // 平衡返回大装甲板
      return kBigArmorWidth;
    else if (type >= 2 && type <= 6)
      return kSmallArmorWidth;
    else
      return kBigArmorWidth;
  }

 private:
  static void CalcPoints();

  static float kLightBarHeight;
  static float kStickerHeight[9];
  static float kStickerWidth[9];
  static float kSmallArmorWidth;
  static float kBigArmorWidth;
  static float kBuffRadius;
  static float kBuffArmorWidth;
  static float kBuffArmorHeight;
  static std::vector<cv::Point3f> robot_world_points_list[10];
  static std::vector<cv::Point3f> buff_world_points_list[5];
  static std::vector<cv::Point3f> buff_disturb_world_points_list;

  static bool is_first_time_;
};

/**
 * @name    ReceiveMessage
 * @brief   从下位机接收的信息
 */
struct ReceiveMessage {
  bool disturb_buff = false;
  bool lock_command = false;
  bool spin_command = false;
  Mode mode = ArmorMode;
  float bulletspeed = 2700.0f;  // cm/s
  int enemy_color = 0;
  ShootPlatform shoot_platform;
  int buff_type = 0;
  bool sentryCommand;

#ifdef DOUBLE_GUN
  bool left = false;
  bool right = false;
#endif
};

/**
 * @brief 将debug界面的二值化处理与灯条检测的二值化处理统一到一处
 * @param
 * @return
 */
void Lightbar_threshold(const cv::Mat& src, int& enemy_color, cv::Mat& dist);

/*
 * @brief: 获得这个m的roi区域，做出判断roi是否超出了m的区域
 * @return:
 * 如果没有超出m的区域，则返回这个m的roi区域，否则返回一个cv::Mat，通过判断empty判断
 *
 * @param:
 *  [ in  ] m :原图
 *  [ out ] roi :感兴趣区域
 */
cv::Mat ROI(const cv::Mat& m, const cv::Rect& roi);

/**
 * @name    CalcDistance
 * @brief   计算两个二维Point距离, 传入的Point可以是任意类型的Point
 * @return  float 二维距离
 */
template <typename T, typename P>
float CalcDistance(cv::Point_<T> point_a, cv::Point_<P> point_b) {
  float distance;
  distance =
      powf((point_a.x - point_b.x), 2) + powf((point_a.y - point_b.y), 2);
  distance = sqrtf(distance);
  return distance;
}

/**
 * @name    CalcDistance
 * @brief   计算两个三维Point距离, 传入的Point可以是任意类型的Point
 * @return  float 三维距离
 */
template <typename T, typename P>
float CalcDistance(cv::Point3_<T> point_a, cv::Point3_<P> point_b) {
  float distance;
  distance = powf((point_a.x - point_b.x), 2) +
             powf((point_a.y - point_b.y), 2) +
             powf((point_a.z - point_b.z), 2);
  distance = sqrtf(distance);
  return distance;
}

/**
 * @brief 把Mat转换为Point3f
 * @param Mat
 * @return Point3f
 */
inline cv::Point3f MatToPoint3f(cv::Mat mat) {
  return cv::Point3f(mat.at<float>(0), mat.at<float>(1), mat.at<float>(2));
}

/**
 * @name    重力补偿
 * @note    单位 厘米，厘米每秒，厘米每平方秒
 * @return  返回重力补偿后的角度
 */
template <typename T>
cv::Vec2f ParabolaSolve(cv::Point_<T> point_a, float kv, float kg = 978.8f) {
  // x1=v*cos@*t
  // y1=v*sin@t-g*t^2/2
  // 联立方程消去t,得关于出射角tan@的方程kg*x1*x1/(2*kv*kv)*tan@^2+x1*tan@+kg*x1*x1/(2*kv*kv)-y1=0
  float x1 = point_a.x, y1 = point_a.y;
  float a = kg * x1 * x1 / (2 * kv * kv), b = -x1,
        c = kg * x1 * x1 / (2 * kv * kv) - y1;
  if (a == 0) {
    std::cout << "a=0" << std::endl;
  }
  float tan_phi0 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a),
        tan_phi1 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);

  if (b * b - 4 * a * c <= 0) {
    return {-1, -1};
  }
  if (kv == 0) {
    kv = 700;
  }
  float phi0 = atan(tan_phi0), phi1 = atan(tan_phi1);
  if (isnan(phi0) || isnan(phi1)) {
    std::cout << "-b:" << -b << " sqrt(b*b-4*a*c): " << sqrt(b * b - 4 * a * c)
              << " (2 * a): " << (2 * a) << std::endl;
    std::cout << "tan_phi0: " << tan_phi0 << " tan_phi1: " << tan_phi1
              << std::endl;
    std::cout
        << "[nan]:发生在base_tookit.h 的ParabolaSolve(),原因：phi0或phi1是nan"
        << std::endl;
  }
  cv::Vec2f ret = {-phi0, -phi1};
  if (b * b - 4 * a * c <= 0) {
    ret = {-atan(y1 / x1), -(atan(y1 / x1) + float(0.001))};
    return ret;
  }
  if (isnan(phi0) || isnan(phi1)) {
    ret = {-atan(y1 / x1), -(atan(y1 / x1) + float(0.001))};
    return ret;
  }
  return ret;
}

/**
 * @brief 角度限制
 * @param [in] angle 需要限制的角度
 * @param [in] type 0: [-pi,pi) 1: [0,2pi)
 * @return double 限制后的角度
 */
double AngleLimit(double angle, uint8_t type = 0);

rclcpp::QoS HighFrequencySensorDataQoS(int depth = 100);

/**
 * @brief 丢弃y轴数据
 * @param Point3f
 * @return Point3f
 */
inline cv::Point3f DropYAxis(cv::Point3f p) { return cv::Point3f(p.x, 0, p.z); }

cv::Vec3f RotationMatrixToEulerAngles(cv::Mat &R, bool flag) ;
}  // namespace tdttoolkit
#endif