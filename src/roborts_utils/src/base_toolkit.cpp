#include "base_toolkit.h"

#include "base_msg.h"

namespace tdttoolkit {
uint64_t Time::frame_id_ = 0;
uint64_t Time::frame_time_ = 0;
std::chrono::_V2::system_clock::time_point Time::init_clock_;
uint64_t Time::time_diff_ = 0;

cv::Mat EulerAnglesToRotationMatrix(cv::Vec3f &euler, bool order) {
  // 绕X轴旋转 旋转矩阵
  cv::Mat R_x = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, cos(euler[0]),
                 -sin(euler[0]), 0, sin(euler[0]), cos(euler[0]));

  // 绕Y轴旋转 旋转矩阵
  cv::Mat R_y = (cv::Mat_<float>(3, 3) << cos(euler[1]), 0, sin(euler[1]), 0, 1,
                 0, -sin(euler[1]), 0, cos(euler[1]));

  // 绕Z轴旋转 旋转矩阵
  cv::Mat R_z = (cv::Mat_<float>(3, 3) << cos(euler[2]), -sin(euler[2]), 0,
                 sin(euler[2]), cos(euler[2]), 0, 0, 0, 1);

  // 按Z-Y-X欧拉角或X-Y-Z欧拉角顺序转化为总的旋转矩阵
  cv::Mat R;

  if (order)
    R = R_z * R_y * R_x;
  else
    R = R_x * R_y * R_z;

  return R;
}

/**
 * @class KalmanFilter
 */

void KalmanFilter::init(float init) {
  setIdentity(statePre, cv::Scalar::all(init));  // 预测值
  setIdentity(statePost, cv::Scalar::all(init));
  setIdentity(errorCovPost, cv::Scalar::all(0.1));  // 后验概率P(k|k)[1x1]
}

KalmanFilter::KalmanFilter() : cv::KalmanFilter(1, 1, 1) {
  setIdentity(statePre, cv::Scalar::all(0));  // 预测值
  setIdentity(statePost, cv::Scalar::all(0));
  setIdentity(transitionMatrix, cv::Scalar::all(1));   // 转移矩阵A[1x1]
  setIdentity(controlMatrix, cv::Scalar::all(1));      // 控制矩阵H[1x1]
  setIdentity(measurementMatrix, cv::Scalar::all(1));  // 测量矩阵H[1x1]
  setIdentity(errorCovPost, cv::Scalar::all(0.1));  // 后验概率P(k|k)[1x1]
}

float KalmanFilter::Estimate(float measurement_value) {
  // 时间连续, 就正常执行滤波过程

  cv::KalmanFilter::predict();
  cv::Mat measurement = cv::Mat_<float>(1, 1) << measurement_value;
  cv::KalmanFilter::correct(measurement);

  return statePost.at<float>(0);
}

/*EKF 实现 */

void EKF::SetQ(float DeltaT) {
  transitionMatrix = (cv::Mat_<float>(4, 4) << 1, DeltaT, 0, 0, 0, 1, 0, 0, 0,
                      0, 1, DeltaT, 0, 0, 0, 1);

  processNoiseCov =
      (cv::Mat_<float>(4, 4) << pow(sigma_ax, 2) * pow(DeltaT, 4) / 4.,
       pow(sigma_ax, 2) * pow(DeltaT, 3) / 2., 0, 0,
       pow(sigma_ax, 2) * pow(DeltaT, 3) / 2.,
       pow(sigma_ax, 2) * pow(DeltaT, 2), 0, 0, 0, 0,
       pow(sigma_az, 2) * pow(DeltaT, 4) / 4.,
       pow(sigma_az, 2) * pow(DeltaT, 3) / 2., 0, 0,
       pow(sigma_az, 2) * pow(DeltaT, 3) / 2.,
       pow(sigma_az, 2) * pow(DeltaT, 2));
}
void EKF::SetR() {
  measurementNoiseCov =
      (cv::Mat_<float>(2, 2) << pow(sigma_yaw, 2), 0, 0, pow(sigma_dis, 2));
}
void EKF::Predict() {
  statePre = transitionMatrix * statePost;

  errorCovPre =
      transitionMatrix * errorCovPost * transitionMatrix.t() + processNoiseCov;
}

void EKF::Correct(const cv::Mat &measurement, bool ifContinous) {
  if (ifContinous) {

    measure_err = measurement - hx();

    convariance = Jacob * errorCovPre * Jacob.t() + measurementNoiseCov;

    gain = errorCovPre * Jacob.t() * convariance.inv();

    statePost = statePre + gain * measure_err;

    errorCovPost = (cv::Mat::eye(4, 4, CV_32F) - gain * Jacob) * errorCovPre;

  } else {
    float r = measurement.at<float>(1, 0), yaw = measurement.at<float>(0, 0),
          px = -r * sin(yaw), pz = r * cos(yaw);
    statePost = (cv::Mat_<float>(4, 1) << px, 0, pz, 0);
    errorCovPost = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 100, 0, 0, 0, 0, 1,
                    0, 0, 0, 0, 100);
  }
  // return statePost;
}

cv::Mat EKF::hx() {
  float px = statePre.at<float>(0, 0),  // 先验估计处进行泰勒展开
      pz = statePre.at<float>(2, 0), dis2 = px * px + pz * pz;
  float yaw = -atan(px / pz);

  if (pz < 0) {
    if (px < 0)
      yaw += CV_PI;
    else
      yaw -= CV_PI;
  }
  cv::Mat tool = (cv::Mat_<float>(2, 1) << yaw, sqrt(dis2));

  Jacob = (cv::Mat_<float>(2, 4) << -pz / dis2, 0, px / dis2,
           0,  // 在上一帧的后验估计进行泰勒展开
           px / sqrt(dis2), 0, pz / sqrt(dis2), 0);

  return tool;
}

/**
 * @class Differential_Evolution 差分进化算法
 */
void Differential_Evolution::Print() {
  for (std::size_t i = 0; i < parentpop.size(); i++) {
    for (int j = 0; j < DIMENSION; j++) {
      TDT_INFO("第%d个变量:", j + 1);
    }
    TDT_INFO("fitness = %lf", parentpop[i].m_fitness);
  }
}

double Differential_Evolution::RandReal(double left, double right) {
  double range = right - left;
  double result(0);
  result = fmod(rand(), range) + left;
  return result;
}

double Differential_Evolution::RandInt(int left, int right) {
  int range = right - left;
  double result(0);
  result = rand() % (range + 1) + left;
  return result;
}

void Differential_Evolution::InitPop() {
  for (int i = 0; i < SIZE; i++) {
    for (int j = 0; j < DIMENSION; j++) {
      parentpop[i].m_xreal[j] =
          RandReal(LOWER_BOUND[j], UPPER_BOUND[j]);  // 将父群体赋随机实数值
    }
    parentpop[i].m_fitness =
        CalculateFitness(parentpop[i].m_xreal);  // 代入函数里进行计算
  }
}

void Differential_Evolution::ParentSelection(std::vector<int> &index) {
  for (int i = 0; i < 3; i++) {
    int r(0);
    bool flag(false);
    do {
      flag = false;
      r = RandInt(0, SIZE - 1);
      for (auto var : index)
        if (r == var) flag = true;
    } while (flag);
    index.push_back(r);
  }
}

void Differential_Evolution::Mutation(std::vector<int> &index, int i) {
  int r1 = index[0], r2 = index[1], r3 = index[2];
  for (int j = 0; j < DIMENSION; j++) {
    // 根据差分变异算子进行变异
    childpop[i].m_xreal[j] =
        parentpop[r1].m_xreal[j] +
        F * (parentpop[r2].m_xreal[j] - parentpop[r3].m_xreal[j]);
    // 如果越界，随机取一个值
    if (childpop[i].m_xreal[j] < LOWER_BOUND[j] ||
        childpop[i].m_xreal[j] > UPPER_BOUND[j])
      childpop[i].m_xreal[j] = RandReal(LOWER_BOUND[j], UPPER_BOUND[j]);
  }
}

void Differential_Evolution::BinCrossover(int i) {
  // 设置一个temp值，保证子个体不会与父个体完全相同
  int temp = RandInt(0, DIMENSION - 1);
  for (int j = 0; j < DIMENSION; j++) {
    // 如果随机产生的RandReal小于CR或者j=r,那么就将变异后的种群放入选择群体中
    if (RandReal(0, 1) > CROSSOVER_RATE && j != temp) {
      childpop[i].m_xreal[j] = parentpop[i].m_xreal[j];
    }
  }
}

void Differential_Evolution::Selection(int i) {
  parentpop[i].m_fitness = CalculateFitness(parentpop[i].m_xreal);
  childpop[i].m_fitness = CalculateFitness(childpop[i].m_xreal);
  if (fabs(childpop[i].m_fitness) <=
      fabs(parentpop[i].m_fitness))  // 保留最小的函数值
    parentpop[i] = childpop[i];
}

std::vector<double> Differential_Evolution::GetDE_Result() {
  std::vector<double> temp;
  srand((unsigned)time(NULL));  // 在生成随机数之前设置一次随机种子
  InitPop();
  for (int i = 0; i < ITER_NUM; i++) {
#pragma omp parallel for
    for (int j = 0; j < SIZE; j++)  // 对每一个个体进行操作
    {
      std::vector<int> index;
      ParentSelection(index);
      Mutation(index, j);
      BinCrossover(j);
      Selection(j);
    }
    // 调试用
    // Print();
  }
  // 迭代完选取fitness最小值作为最优结果输出
  double minFitness = 100000;
  int index;
  for (std::size_t i = 0; i < parentpop.size(); i++) {
    if (fabs(parentpop[i].m_fitness) < minFitness) {
      minFitness = fabs(parentpop[i].m_fitness);
      index = i;
    }
  }
  for (int j = 0; j < DIMENSION; j++) {
    TDT_INFO(" 22222 最优拟合结果：第%d个变量:%lf", j + 1,
             parentpop[index].m_xreal[j]);
    temp.push_back(parentpop[index].m_xreal[j]);
  }
  return temp;
}

cv::Mat PolyFit(std::vector<float> array_x, std::vector<float> array_y,
                int size, int n) {
  // 所求未知数个数
  int x_num = n + 1;  // 拟合方程阶数
  // 构造矩阵U和Y
  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);

  for (int i = 0; i < mat_u.rows; ++i)
    for (int j = 0; j < mat_u.cols; ++j) {
      mat_u.at<double>(i, j) = pow(array_x[i], j);
    }

  for (int i = 0; i < mat_y.rows; ++i) {
    mat_y.at<double>(i, 0) = array_y[i];
  }

  // 矩阵运算，获得系数矩阵K
  cv::Mat mat_k(x_num, 1, CV_64F);
  mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
  return mat_k;
}

cv::Point2f Bezier(std::vector<cv::Point2f> points, float t) {
  if (points.size() == 2) return points[0] + t * (points[1] - points[0]);

  std::vector<cv::Point2f> points_temp;
  for (std::size_t i = 0; i < points.size() - 1; i++) {
    points_temp.push_back(points[i] + t * (points[i + 1] - points[i]));
  }
  return Bezier(points_temp, t);
}

float integral(std::vector<cv::Point2f> speedPoints) {
  float goal = 0;
  for (std::size_t i = 0; i < speedPoints.size() - 1; i++) {
    goal += 0.5 * (speedPoints[i].y + speedPoints[i + 1].y) *
            (speedPoints[i + 1].x - speedPoints[i].x) / 1e3;
  }
  return goal;
}

Eigen::MatrixXf regress(std::vector<float> x, std::vector<float> y) {
  Eigen::MatrixXf oneMat;
  oneMat.setOnes(x.size(), 1);
  Eigen::VectorXf X =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(x.data(), x.size());
  Eigen::MatrixXf X_;
  X_.resize(x.size(), 2);  //
  X_ << oneMat, X;
  Eigen::VectorXf Y_;
  Y_ = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(y.data(), y.size());

  Eigen::MatrixXf K = X_.colPivHouseholderQr().solve(Y_);
  return K;
}

size_t MatHash(const cv::Mat &mat) {
  std::string str;
  cv::MatConstIterator_<uchar> it = mat.begin<uchar>(),
                               it_end = mat.end<uchar>();
  for (; it != it_end; ++it) {
    str.append(std::to_string(*it));
  }
  // 计算字符串的哈希值
  boost::hash<std::string> str_hash;
  return str_hash(str);
}

EulerAngles ToEulerAngles(Quaternion q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    angles.pitch =
        std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

void PID::init(double Kp, double Ki, double Kd, double dt, double max,
               double min, double imax) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->dt = dt;
  this->max = max;
  this->min = min;
  this->pre_error = 0;
  this->integral = 0;
  this->imax = imax;
}

double PID::calculate(double setpoint, double pv) {
  double error = setpoint - pv;
  this->integral += error * dt;
  double derivative = (error - pre_error) / dt;
  if (Ki * integral > imax) integral = 0;
  double output = Kp * error + Ki * integral + Kd * derivative;
  if (output > max) output = max;
  if (output < min) output = min;
  pre_error = error;
  return output;
}

bool Rodrigues(cv::InputArray _src, cv::OutputArray _dst,
               cv::OutputArray _jacobian) {
  cv::Mat src = _src.getMat();
  cv::Rodrigues(_src, _dst, _jacobian);
  return 1;
}

Polar3f RectangularToPolar(cv::Point3f const &rectangular_point) {
  Polar3f polar_point;
  polar_point.distance =
      static_cast<float>(cv::norm(cv::Mat(rectangular_point, CV_32F)));
  polar_point.yaw = -atan(rectangular_point.x / rectangular_point.z);

  if (rectangular_point.z < 0) {
    if (rectangular_point.x < 0)
      polar_point.yaw += CV_PI;
    else
      polar_point.yaw -= CV_PI;
  }

  polar_point.pitch = asin(rectangular_point.y / polar_point.distance);
  if (polar_point.distance == 0) {
    // 因为polar_point.distance为0,所以polar_point.pitch->nan
    std::cout << "[nan]:发生在base_tookit.cpp 的RectangularToPolar()"
              << std::endl;
    // assert(polar_point.distance != 0);
  }
  return polar_point;
}

void AngleCorrect(float &angle) {
  while (angle > CV_PI) angle -= (2 * CV_PI);
  while (angle < -CV_PI) angle += (2 * CV_PI);
}


cv::Point3f PolarToRectangular(const Polar3f &polar_point) {
  cv::Point3f rectangular_point;
  rectangular_point.y = sin(polar_point.pitch) * polar_point.distance;
  float r = cos(polar_point.pitch) * polar_point.distance;
  rectangular_point.x = -r * sin(polar_point.yaw);
  rectangular_point.z = r * cos(polar_point.yaw);

  return rectangular_point;
}

void Lightbar_threshold(const cv::Mat &src, int &enemy_color, cv::Mat &dist) {
  cv::Mat img_gray_1, img_gray_2, img_gray_3;
  //        std::vector<cv::Mat> rgb;
  //        split(src, rgb);
  cv::cvtColor(src, img_gray_3, cv::COLOR_BGR2GRAY);
  dist = img_gray_3;
}

cv::Mat ROI(const cv::Mat &m, const cv::Rect &roi) {
  if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols &&
      0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows) {
    return m(roi);
  } else {
    return cv::Mat();
  }
}
std::vector<cv::Point3f> WorldPointLists::robot_world_points_list[10];
std::vector<cv::Point3f> WorldPointLists::buff_world_points_list[5];
std::vector<cv::Point3f> WorldPointLists::buff_disturb_world_points_list;

float WorldPointLists::kLightBarHeight = 5.7;
float WorldPointLists::kStickerHeight[9] = {0,    10.2, 10.1, 10.3, 9.4,
                                            10.2, 10.2, 10.2, 10.2};
float WorldPointLists::kStickerWidth[9] = {0,   4.4, 6.6, 6.3, 7.2,
                                           6.4, 6.5, 6.5, 6.5};
float WorldPointLists::kSmallArmorWidth = 12.9;
float WorldPointLists::kBigArmorWidth = 22.5;
float WorldPointLists::kBuffRadius = 70;
float WorldPointLists::kBuffArmorWidth = 22;
float WorldPointLists::kBuffArmorHeight = 15;

bool WorldPointLists::is_first_time_ = true;

void WorldPointLists::CalcPoints() {
  //  从上到下 依次是：
  //  左灯条 { 0上定点, 1下顶点, 2中心点 }
  //  数字贴纸 { 3左中心点, 4右中心点, 5上中心点, 6下中心点, 7中心点 }
  //  右灯条 { 8上定点, 9下顶点, 10中心点 }

  const std::vector<cv::Point3f> world_points[10] = {
      ///----None---///
      {cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[0] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[0] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[0] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[0] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f)},

      ///---Hero---///
      {cv::Point3f(-kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kBigArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[1] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[1] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[1] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[1] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kBigArmorWidth / 2, 0.f, 0.f)},

      ///---Engineer---///
      {cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[2] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[2] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[2] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[2] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f)},

      ///---Infantry3---///
      {cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[3] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[3] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[3] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[3] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f)},

      ///---Infantry4---///
      {cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[4] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[4] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[4] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[4] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f)},

      ///---Infantry5---///
      {cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[5] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[5] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[5] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[5] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f)},

      ///---OUTPOST---///
      {
          cv::Point3f(-kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(-kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(-kBigArmorWidth / 2, 0.f, 0.f),
          // cv::Point3f(-kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(0.f, -kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, 0.f, 0.f),
          cv::Point3f(kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(kBigArmorWidth / 2, 0.f, 0.f),
      },

      ///---SENTRY---///
      {
          cv::Point3f(-kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(-kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(-kSmallArmorWidth / 2, 0.f, 0.f),
          // cv::Point3f(-kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(0.f, -kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, 0.f, 0.f),
          cv::Point3f(kSmallArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(kSmallArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(kSmallArmorWidth / 2, 0.f, 0.f),
      },

      ///---Base---///
      {
          cv::Point3f(-kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(-kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(-kBigArmorWidth / 2, 0.f, 0.f),
          // cv::Point3f(-kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(kStickerWidth[6] / 2, 0.f, 0.f),
          // cv::Point3f(0.f, -kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, kStickerHeight[6] / 2, 0.f),
          // cv::Point3f(0.f, 0.f, 0.f),
          cv::Point3f(kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
          cv::Point3f(kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
          cv::Point3f(kBigArmorWidth / 2, 0.f, 0.f),
      },

      ///--Balance--///
      {cv::Point3f(-kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(-kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(-kBigArmorWidth / 2, 0.f, 0.f),
       //  cv::Point3f(-kStickerWidth[1] / 2, 0.f, 0.f),
       //  cv::Point3f(kStickerWidth[1] / 2, 0.f, 0.f),
       //  cv::Point3f(0.f, -kStickerHeight[1] / 2, 0.f),
       //  cv::Point3f(0.f, kStickerHeight[1] / 2, 0.f), cv::Point3f(0.f, 0.f,
       //  0.f),
       cv::Point3f(kBigArmorWidth / 2, -kLightBarHeight / 2, 0.f),
       cv::Point3f(kBigArmorWidth / 2, kLightBarHeight / 2, 0.f),
       cv::Point3f(kBigArmorWidth / 2, 0.f, 0.f)}};

  for (int i = 0; i < 10; i++) {
    robot_world_points_list[i] = world_points[i];
  }

  // 构造能量机关点
  for (int i = 0; i < 5; i++) {
    if (i == 0) {
      //                EBWorldPoints_List[i]={Point3f(0,0,0),Point3f(0,kEBr,14.5f)};
      buff_world_points_list[0] = {
          cv::Point3f(0, 0, 0), cv::Point3f(0, kBuffRadius, 0),
          cv::Point3f(kBuffArmorWidth / 2, kBuffArmorHeight / 2 + kBuffRadius,
                      0),
          cv::Point3f(-kBuffArmorWidth / 2, kBuffArmorHeight / 2 + kBuffRadius,
                      0)};
      //                buff_world_points_list[0]={Point3f(0,0,0),Point3f(0,kBuffRadius,14.5),Point3f(-kBuffArmorWidth/2,kBuffArmorHeight/2+kBuffRadius,14.5),Point3f(kBuffArmorWidth/2,kBuffArmorHeight/2+kBuffRadius,14.5)};
    } else {
      //                EBWorldPoints_List[i]={Point3f(-kEBAMwidth/2,kEBr,14.5),Point3f(kEBAMwidth/2,kEBr,14.5)};
      buff_world_points_list[i] = {cv::Point3f(0, kBuffRadius, 14.5)};

      for (int j = 0; j < buff_world_points_list[i].size(); j++) {
        cv::Vec3f diffrot = {0, 0,
                             float(-i) * 2 * 3.1415926f / 5};  // 旋转的roll角度
        cv::Mat rotmatix = EulerAnglesToRotationMatrix(diffrot, true);
        cv::Point3f tmpp3 = buff_world_points_list[i][j];
        cv::Mat p =
            rotmatix * cv::Mat(cv::Point3f(tmpp3));  // 物体坐标系中点的位置
        buff_world_points_list[i][j] = MatToPoint3f(p);  // 装甲版中心点
      }
    }
  }

  // 构造反符世界点
  buff_disturb_world_points_list = {
      cv::Point3f(0, 0, 0),
      cv::Point3f(kBuffArmorWidth / 2, kBuffArmorHeight / 2, 0),
      cv::Point3f(-kBuffArmorWidth / 2, kBuffArmorHeight / 2, 0),
      cv::Point3f(-kBuffArmorWidth / 2, -kBuffArmorHeight / 2, 0),
      cv::Point3f(kBuffArmorWidth / 2, -kBuffArmorHeight / 2, 0)};
}

double AngleLimit(double angle, uint8_t type) {
  switch (type) {
    case 0:
      angle = fmod(angle + M_PI, 2 * M_PI);
      if (angle < 0) {
        angle += 2 * M_PI;
      }
      return angle - M_PI;
    case 1:
      angle = fmod(angle, 2 * M_PI);
      if (angle < 0) {
        angle += 2 * M_PI;
      }
      return angle;
    default:
      TDT_WARNING("Unknown type of angle limit");
      angle = fmod(angle + M_PI, 2 * M_PI);
      if (angle < 0) {
        angle += 2 * M_PI;
      }
      return angle - M_PI;
  }
}

float CalcAngleDifference(float a, float b) { return AngleLimit(a - b); }

rclcpp::QoS HighFrequencySensorDataQoS(int depth) {
  auto qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = depth;
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile));
}
cv::Vec3f RotationMatrixToEulerAngles(cv::Mat &R, bool flag) {
  R.convertTo(R, CV_32F);
  // assert(RotationMatrixJudgement(R));

  // flag = true：Z-Y-X欧拉角
  if (flag) {
    float cosy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) +
                      R.at<float>(1, 0) * R.at<float>(1, 0));
    bool singular = cosy < 1e-6;
    float x, y, z;

    if (!singular) {
      x = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
      y = atan2(-R.at<float>(2, 0), cosy);
      z = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
    } else {
      x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
      y = atan2(-R.at<float>(2, 0), cosy);
      z = 0;
    }
    return cv::Vec3f(x, y, z);  // 逆正顺负
  }

  // flag = false：Z-X-Y欧拉角
  else {
    float cosx = sqrt(R.at<float>(0, 1) * R.at<float>(0, 1) +
                      R.at<float>(1, 1) * R.at<float>(1, 1));
    bool singular = cosx < 1e-6;
    float x, y, z;

    if (!singular) {
      x = atan2(R.at<float>(2, 1), cosx);
      y = atan2(-R.at<float>(2, 0), R.at<float>(2, 2));
      z = atan2(-R.at<float>(0, 1), R.at<float>(1, 1));
    } else {
      x = atan2(R.at<float>(2, 1), cosx);
      y = atan2(-R.at<float>(2, 0), R.at<float>(2, 2));
      z = 0;
    }
    return cv::Vec3f(x, y, z);  // 逆正顺负
  }
}

}  // namespace tdttoolkit