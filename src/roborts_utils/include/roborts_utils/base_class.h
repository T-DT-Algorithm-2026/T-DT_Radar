/**
 * @Name: file name
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-11-16 11:32:32
 * @LastEditors: your name
 * @LastEditTime: 2019-11-16 16:00:04
 */

#ifndef __BASE_CLASS_H
#define __BASE_CLASS_H

#include "base_toolkit.h"

namespace tdttoolkit {

/**
 * @name     自定义矩形类
 * @brief    一个自定义结构体,对原本的旋转矩阵的属性进行包装
 * @features 包装完之后width总是长边,height总是短边
 *           angle总是矩形旋转到width与x轴平行所转过的角度 0~180
 *           顶点编号总是左下角为0顺时针标到3
 * @refer 旋转矩形的angle参考:https://www.cnblogs.com/panxiaochun/p/5478555.html
 *           旋转矩形的points参考:https://blog.csdn.net/mailzst1/article/details/83141632
 */
class CustomRect {
 public:
  /**
   * 用来构造一个空的类
   */
  CustomRect() = default;
  /**
   * 从RotatedRect来的构造函数
   * @param rRect 用来构造的RotatedRect
   */
  explicit CustomRect(cv::RotatedRect &r_rect);
  /**
   * 从点集构造,最小外接矩形
   * @param inputContour 用来构造的点集
   */
  explicit CustomRect(const std::vector<cv::Point> &input_contour);

  CustomRect(const cv::Point2f &center, const cv::Size2f &size, float angle);
  CustomRect(const cv::Point &center, const cv::Size &size, float angle);

  CustomRect(const CustomRect &c_rect) = default;

  inline cv::Size2f GetSize() const { return this->size_; };
  inline void SetSize(cv::Size const &size) {
    this->size_ = size;
    FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
    UpdateVertices();
  };
  inline float GetAngle() const { return this->angle_; };
  // 这东西都没有函数调用为什么会有后续的方法呢？

  inline void SetAngle(float angle) {
    this->angle_ = angle;
    UpdateVertices();
  };
  ;
  inline void SetCenter(cv::Point2d const &center) {
    this->center_ = center;
    UpdateVertices();
  };
  // 最小高度差
  inline float Vertical() const {
    return fmax(this->bl_.y, this->br_.y) - fmin(this->tl_.y, this->tr_.y);
  };
  // 最小宽度差
  inline float Cross() const {
    return fmax(this->tr_.x, this->br_.x) - fmin(this->tl_.x, this->bl_.x);
  };
  inline float GetWidth() const { return this->size_.width; };
  inline float GetHeight() const { return this->size_.height; };
  // 计算灯条的面积，用于大小排序，仅用于增强开火决策跟踪效果
  inline float GetArea() const { return this->size_.area(); };
  inline std::vector<cv::Point> GetVertices() const {
    return {this->bl_, this->tl_, this->tr_, this->br_};
  };
  inline std::vector<cv::Point2f> GetVertices2f() const {
    return {this->bl_, this->tl_, this->tr_, this->br_};
  };
  inline cv::Rect GetRect() const {
    return cv::boundingRect(
        std::vector<cv::Point2i>({this->bl_, this->tl_, this->tr_, this->br_}));
  };
  inline cv::RotatedRect GetRotRect() const {
    return cv::minAreaRect(
        std::vector<cv::Point2i>({this->bl_, this->tl_, this->tr_, this->br_}));
  };
  inline cv::Point GetCenter() const { return this->center_; };
  inline cv::Point2f GetCenter2f() const { return this->center_; };
  inline cv::Point2f GetTl() const { return this->tl_; };
  inline cv::Point2f GetBl() const { return this->bl_; };
  inline cv::Point2f GetTr() const { return this->tr_; };
  inline cv::Point2f GetBr() const { return this->br_; };

 protected:
  /**
   * @brief 通过size, center, angle 来计算出 四个顶点
   */
  void UpdateVertices();

  inline void NewToOld(cv::RotatedRect &r_rect) {
    if (this->angle_ != 0) {
      this->center_.x = r_rect.center.x;
      this->center_.y = r_rect.center.y;
      this->size_.width = r_rect.size.height;
      this->size_.height = r_rect.size.width;
      this->angle_ -= 90;
    }
  }

  inline void FixWidthAndHeight(float &width, float &height, float &angle) {
    if (width < height) {
      angle = angle - 90;
      // 交换两个数
      float tmp;
      tmp = height;
      height = width;
      width = tmp;
    }
    angle = fabs(angle_);  // 取绝对值
  }

  // 外部可读可写变量
  cv::Size2f size_ = cv::Size2f(0, 0);
  float angle_ = 0;
  cv::Point2f center_ = cv::Point2f(0, 0);

  // 外部只读变量
  /**
   * 四个顶点
   */
  cv::Point2f tl_ = cv::Point2f(0, 0);
  cv::Point2f bl_ = cv::Point2f(0, 0);
  cv::Point2f tr_ = cv::Point2f(0, 0);
  cv::Point2f br_ = cv::Point2f(0, 0);

 public:
  /**
   * @brief 找出点集在对应angle角度的最小外接矩形
   * @param _points 点集
   * @param angle 角度
   * @return 矩形
   */
  static CustomRect minCustomRect(cv::InputArray _points, float angle);
};

/**
 * @name    RobotArmor
 * @brief   机器人车上的装甲板, 封装了从识别部分可直接获取的参数
 */
class RobotArmor {
 public:
  RobotArmor() = default;
  RobotArmor(const CustomRect &sticker_rect, const RobotType &robot_type,
             const std::vector<cv::Point2f> &image_point_list,
             const int &believable);
  RobotArmor(const CustomRect &sticker_rect, const RobotType &robot_type,
             const std::vector<cv::Point2f> &image_point_list,
             const int &believable, int balance[3]);
  RobotArmor(const CustomRect &sticker_rect, const DPRobotType &DP_robot_type,
             const std::vector<cv::Point2f> &image_point_list,
             const cv::Point2f &axis, const int &believable, const float &conf);
  inline RobotType GetRobotType() const { return robot_type_; }
  inline DPRobotType GetDPRobotType() const { return DP_robot_type_; }
  inline CustomRect GetStickerRect() const { return sticker_rect_; }
  inline std::vector<cv::Point2f> GetImagePointsList() const {
    return image_point_list_;
  }
  inline std::vector<cv::Point3f> GetWorldPointsList() const {
    return world_point_list_;
  }
  inline int GetBelievable() const { return believable_; }
  inline void SetRobotType(RobotType robot_type) { robot_type_ = robot_type; }
  inline void SetDPRobotType(DPRobotType DP_robot_type) {
    DP_robot_type_ = DP_robot_type;
  }
  inline void Settimes(int times) { times_ = times; }
  inline void Setaxis(cv::Point2f axis) { axis_ = axis; }
  inline cv::Point2f Getaxis() const { return axis_; }
  inline void Setrect(cv::Rect rect) { rect_ = rect; }
  inline cv::Rect Getrect() const { return rect_; }
  inline int Gettimes() const { return times_; }
  inline float Getconf() const { return conf_; }
  void SetWorldPoints(DPRobotType DProbot_type, int balance[3]);
  void SetWorldPoints(DPRobotType DProbot_type, bool isBalance);
  inline void setBalance(bool balance) { isBalance = balance; }
  inline bool IsBalance() const { return isBalance; }
  inline void SetCenterCam(cv::Point3f center_Cam) {
    this->center_Cam = center_Cam;
  }
  inline cv::Point3f GetCenterCam() const { return center_Cam; }
  inline void SetCenterLivox(cv::Point3f center_Livox) {
    this->center_Livox = center_Livox;
  }
  inline cv::Point3f GetCenterLivox() const { return center_Livox; }
  inline void SetCamId(int cam_id) { this->cam_id = cam_id; }
  inline int GetCamId() const { return cam_id; }
  inline void SetSolved(bool solved) { this->resolved = solved; }
  inline int GetSolved() const { return resolved; }
  inline cv::Point3f GetCenterRobot() const { return center_Robot; }
  inline void SetCenterRobot(cv::Point3f center_Robot) {
    this->center_Robot = center_Robot;
  }

 private:
  RobotType robot_type_ = TYPEUNKNOW;
  DPRobotType DP_robot_type_ = No;
  CustomRect
      sticker_rect_;  // 在检测器里获取时就要融合get_armor_rotatedrect这个函数
  std::vector<cv::Point3f>
      world_point_list_;  // Calc时根据RobotType获取对应的三维坐标
  std::vector<cv::Point2f> image_point_list_;  // Calc时算出
  int believable_ = 0;
  cv::Point2f axis_;
  cv::Rect rect_;  // 装甲板对应车辆的框，前提是车辆有装甲板子
  int times_;
  float conf_;
  bool isBalance = false;
  bool resolved = false;
  int cam_id;
  cv::Point3f center_Cam;
  cv::Point3f center_Livox;
  cv::Point3f center_Robot;
};

/**
 * @name    ResolvedArmor
 * @brief   封装了单个装甲板的解算结果, 为RobotPredictor提供必要数据
 */
class ResolvedArmor {
 public:
  ResolvedArmor(RobotType const &robot_type, CustomRect const &sticker_rect,
                cv::Rect const &armor_bounding_rect, cv::Mat position_in_world,
                cv::Vec3f const &euler_world_to_object,
                Polar3f const &polar_in_world, cv::Mat pnp_tvec,
                cv::Mat pnp_rvec, std::vector<cv::Point2f> image_points,
                std::vector<cv::Point3f> world_points, cv::Point2f axis,
                int believable, cv::Vec3f TvecCameraInWorld_, float conf,
                float lightCenter_dis);

  ResolvedArmor(const ResolvedArmor &obj) {
    this->robot_type_ = obj.robot_type_;      // 机器人种类
    this->sticker_rect_ = obj.sticker_rect_;  // 贴纸区域矩形
    this->position_in_world_ =
        obj.position_in_world_.clone();  // 装甲板中心点在世界系中的三维坐标
    this->euler_world_to_object_ =
        obj.euler_world_to_object_;  // 世界系->物体系 Z-X-Y欧拉角
    this->polar_in_world_ =
        obj.polar_in_world_;  // 装甲板中心点在世界系中的极坐标
    this->pnp_tvec_ =
        obj.pnp_tvec_.clone();  // solevepnp求得的平移向量与旋转向量
    this->pnp_rvec_ = obj.pnp_rvec_.clone();
    this->image_points_ = obj.image_points_;
    this->world_points_ = obj.world_points_;
    this->armor_bounding_rect_ = obj.armor_bounding_rect_;
    this->tag_ = obj.tag_;
    this->believable_ = obj.believable_;
    this->axis_ = obj.axis_;
    this->RVec2World_ = obj.RVec2World_.clone();
    this->TVecInWorld_ = obj.TVecInWorld_.clone();
    this->TvecCameraInWorld = obj.TvecCameraInWorld;
    this->conf_ = obj.conf_;
    this->lightCenter_dis_ = obj.lightCenter_dis_;
  }
  ResolvedArmor();

  inline RobotType GetRobotType() const { return robot_type_; }
  inline CustomRect GetStickerRect() const { return sticker_rect_; }
  inline cv::Mat GetPositionInWorld() const { return position_in_world_; }
  inline void SetPositionInWorld(cv::Mat position_in_world) {
    position_in_world_ = position_in_world.clone();
  }
  inline cv::Vec3f GetEuler() const { return euler_world_to_object_; }
  inline Polar3f GetPolar() const { return polar_in_world_; }
  inline std::vector<cv::Point2f> GetImagePoints() const {
    return image_points_;
  }
  inline std::vector<cv::Point3f> GetWorldPoints() const {
    return world_points_;
  }
  inline cv::Mat GetPNPTvec() const { return pnp_tvec_; }
  inline cv::Mat GetPNPRvec() const { return pnp_rvec_; }
  inline cv::Rect GetBoundingRect() const { return armor_bounding_rect_; }
  inline void SetTag(int tag) { tag_ = (tag + 4) % 4; }
  inline int GetTag() const { return tag_; }
  inline int GetBelievable() const { return believable_; }
  inline void SetRVecToWorld(cv::Mat RVec2World) {
    this->RVec2World_ = RVec2World.clone();
  }
  inline void SetTVecInWorld(cv::Mat TVecInWorld) {
    this->TVecInWorld_ = TVecInWorld.clone();
  }
  inline cv::Mat GetRVecToWorld() const { return this->RVec2World_; }
  inline cv::Mat GetTVecInWorld() const { return this->TVecInWorld_; }
  inline void SetRobotType(RobotType type) { robot_type_ = type; }
  inline bool GetResolvedStatus() { return resolved; }
  inline cv::Point2f Getaxis() const { return axis_; }
  inline cv::Mat GetTvecCameraInWorld() { return TvecCameraInWorld; }
  inline float Getconf() { return conf_; }
  inline float GetlightCenter_dis_() { return lightCenter_dis_; };

 private:
  RobotType robot_type_;       // 机器人种类
  CustomRect sticker_rect_;    // 贴纸区域矩形
  cv::Mat position_in_world_;  // 装甲板中心点在世界系中的三维坐标
  cv::Vec3f euler_world_to_object_;  // 世界系->物体系 Z-X-Y欧拉角
  Polar3f polar_in_world_;  // 装甲板中心点在世界系中的极坐标
  cv::Mat pnp_tvec_;        // solevepnp求得的平移向量与旋转向量
  cv::Mat pnp_rvec_;
  std::vector<cv::Point2f> image_points_;
  std::vector<cv::Point3f> world_points_;
  cv::Point2f axis_;
  cv::Rect armor_bounding_rect_;
  int tag_ = -1;
  int believable_ = 0;  // 0:无灯条 1:双灯条 2:左灯条 3:右灯条
  cv::Mat RVec2World_;
  cv::Mat TVecInWorld_;
  bool resolved = 0;
  cv::Mat TvecCameraInWorld;
  float conf_;
  float lightCenter_dis_;
};

/**
 * @name    Robot
 * @brief   封装了一辆机器人
 */
class Robot {
 public:
  Robot();

  Robot(std::vector<ResolvedArmor> const &resolved_armors,
        RobotType const &robot_type, uint64_t const &time_stamp);
  Robot(ResolvedArmor &resolved_armor, RobotType const &robot_type,
        uint64_t const &time_stamp);
  inline std::vector<ResolvedArmor> &GetResolvedArmors() {
    return resolved_armors_;
  }
  inline long GetTimeStamp() const { return long(time_stamp_); }
  inline RobotType GetRobotType() const { return robot_type_; }
  inline cv::Mat GetTvecCameraInWorld() const { return tvecCameraInWorld; }
  inline cv::Point2f GetAxisInPixel() const { return axisInPixel; }
  inline void AddResolvedArmors(ResolvedArmor &resolved_armor) {
    resolved_armors_.push_back(resolved_armor);
  }
  inline void SetTimeStamp(uint64_t time_stamp) { time_stamp_ = time_stamp; }
  void swap(Robot &b) {
    Robot c = *this;
    *this = b;
    b = c;
  }
  inline void SetRobotType(tdttoolkit::RobotType a) { robot_type_ = a; }

 private:
  RobotType robot_type_;
  uint64_t time_stamp_;  // 单位:ms
  std::vector<ResolvedArmor> resolved_armors_;
  cv::Mat tvecCameraInWorld;
  cv::Point2f axisInPixel;
};
/**
 * @name    BuffArmor
 */
class BuffArmor {
 public:
  BuffArmor() = default;
  explicit BuffArmor(CustomRect const &buff_rect);

  inline void SetTimeNow(float time_now) { time_now_ = time_now; }
  inline float GetTimeNow() const { return time_now_; }
  inline void SetRRect(cv::Rect &r_rect) { R_rect_ = r_rect; }
  inline cv::Rect GetRRect() { return R_rect_; }
  inline void SetAngle(float angle) { rotate_angle_ = angle; }
  inline float GetAngle() const { return rotate_angle_; }
  inline void SetEmpty(bool empty) { empty_ = empty; }
  inline bool GetEmpty() const { return empty_; }
  inline void SetCirclePoint(cv::Point2f &cir_point2f) {
    circle_point_ = cir_point2f;
  }
  inline cv::Point2f GetCirclePoint() const { return circle_point_; }
  inline void SetTlrPoint(cv::Point2f tl, cv::Point2f tr) {
    tl_point_ = tl;
    tr_point_ = tr;
  };
  inline void SetDlrPoint(cv::Point2f bl, cv::Point2f br) {
    bl_point_ = bl;
    br_point_ = br;
  }
  inline cv::Point2f GettlPoint() const { return tl_point_; };
  inline cv::Point2f GettrPoint() const { return tr_point_; };
  inline cv::Point2f GetblPoint() const { return bl_point_; };
  inline cv::Point2f GetbrPoint() const { return br_point_; };
  inline void SetBuffType(BuffType buff_type) {
    buff_type_ = buff_type;
  }  // 流水灯还是已激活
  inline BuffType GetBuffType() const { return buff_type_; }

  inline void SetBuffCenter(const cv::Point2f &buff_center) {
    buff_center_ = buff_center;
  }
  inline cv::Point2f GetBuffCenter() const { return buff_center_; }
  inline CustomRect GetBuffRect() const { return buff_rect_; }
  inline void SetPoints(std::vector<cv::Point2f> Points) {
    Buff_Points_ = Points;
  }
  inline std::vector<cv::Point2f> GetPoints() const { return Buff_Points_; }

 private:
  float time_now_ = 0.;
  bool empty_ = true;
  CustomRect buff_rect_;
  cv::Rect R_rect_;
  float is_r_ = 2.;
  float rotate_angle_ = 0.;
  float svm_flow_water_ = 2;  // 1是流水灯,0是非流水
  cv::Mat single_R_;
  cv::Mat single_T_;
  // cv::Point3f world_center_;
  float kd_flow_water_ = 100;
  cv::Point2f circle_point_;
  cv::Point2f tl_point_;     // 装甲板左上点
  cv::Point2f tr_point_;     // 装甲板右上点
  cv::Point2f bl_point_;     // 装甲板左下点
  cv::Point2f br_point_;     // 装甲板右下点
  cv::Point2f buff_center_;  // 装甲板中心点
  BuffType buff_type_ = None;
  std::vector<cv::Point2f> Buff_Points_;
};

/**
 * @name    Buff2
 * @name    Buff2 能量机关
 * @brief
 * @note
 */
class Buff2 {
 public:  // 待定
  Buff2();
  Buff2(std::vector<BuffArmor> const &buff_armor,
        cv::Mat const &vec_rvec_object_to_camera,
        cv::Mat const &tvec_object_in_camera,
        //             cv::Mat const &pnp_rvec_object_to_camera,
        //             cv::Mat const &pnp_tvec_object_in_camera,
        cv::Mat const &R_rvec_object_in_camera,
        cv::Mat const &R_tvec_object_in_camera);
  bool disturbbuff = false;

  inline std::vector<BuffArmor> GetBuffArmors() { return buff_armors_; }
  inline Polar3f GetPolar3f() { return target_world_polar; }
  inline cv::Point3f GetWorldPoint() { return target_world_point; }

  inline cv::Mat GetRvec() { return vec_rvec_object_to_camera_; }
  inline cv::Mat GetTvec() { return tvec_object_in_camera_; }
  //        inline cv::Mat GetPnpRvec() { return pnp_rvec_object_to_camera_; }
  //        inline cv::Mat GetPnpTvec() { return pnp_tvec_object_in_camera_; }
  inline cv::Mat GetRrvec() { return R_rvec_object_in_camera_; }
  inline cv::Mat GetRTvec() { return R_tvec_object_in_camera_; }

  inline int GetFinal() { return final_index; }
  inline bool GetEmpty() { return empty_; }

 private:
  int buff_armor_flow_;
  tdttoolkit::Polar3f target_world_polar;
  cv::Point3f target_world_point;
  std::vector<BuffArmor> buff_armors_;
  std::vector<BuffArmor> history_armor_;
  cv::Mat vec_rvec_object_to_camera_;
  cv::Mat tvec_object_in_camera_;
  //        cv::Mat                pnp_rvec_object_to_camera_;
  //        cv::Mat                pnp_tvec_object_in_camera_;
  cv::Mat R_rvec_object_in_camera_;
  cv::Mat R_tvec_object_in_camera_;
  bool empty_ = true;
  int final_index = 0;

  /**
   * @name CompleteBuff
   * #brief  完整的能量Buff的构建
   */
  void CompleteBuff();
};

enum beatMode { ARMOR_MODE = 0, ENERGY_MODE = 1 };

class MatchInfo {
 public:
  uint8_t self_color = 0;  // 0 蓝 1 红
  int16_t match_time =
      -100;  // 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100
             // 若未连接至裁判系统发送-200
  int robot_hp[14];  // 0-6 1-7号敌方机器人血量 7-13 1-7号己方机器人血量
  uint8_t supper_electric_capacity_status = -1;  // 超级电容状态 0-100 可用度

  int remaining_recoverable_health = 600;

  MatchInfo(){};
  MatchInfo(uint8_t self_color, int16_t match_time, uint8_t robot_hp[14],
            uint8_t supper_electric_capacity_status) {
    this->self_color = self_color;
    this->match_time = match_time;
    for (int i = 0; i < 14; i++) {
      this->robot_hp[i] = robot_hp[i] * 5;
    }
    this->supper_electric_capacity_status = supper_electric_capacity_status;
  }
};

class NavStatus {
 public:
  Vec2d nav_pos;
  Vec2d nav_target;
  bool on_arrive = 0;

  int specific_target = 0;
  enum SpecificTarget {
    NONE = 0,
    SUPPLY = 1,
  };
};

}  // namespace tdttoolkit

#endif  // __BASE_CLASS_H
