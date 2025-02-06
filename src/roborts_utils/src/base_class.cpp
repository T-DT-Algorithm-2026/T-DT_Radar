/**
 * @Name: base_class.cpp
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-11-16 11:32:32
 * @LastEditors: your name
 * @LastEditTime: 2019-11-16 16:00:04
 */
#include "base_class.h"

#include "base_toolkit.h"
// #include "tdtmsg.h"
#include <opencv2/core/types.hpp>

#include "base_msg.h"

using namespace cv;
using namespace std;

namespace tdttoolkit {

/**
 * @class   CustomRect
 */

CustomRect::CustomRect(cv::RotatedRect &r_rect) {
  this->size_ = r_rect.size;
  this->angle_ = r_rect.angle;
  this->center_ = r_rect.center;
  if (cv::getVersionMinor() >= 5) {
    NewToOld(r_rect);
  }
  FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
  this->UpdateVertices();
}

CustomRect::CustomRect(const std::vector<cv::Point> &input_contour) {
  // 可以获得这个轮廓的最小旋转矩形
  cv::RotatedRect r_rect = cv::minAreaRect(input_contour);
  this->size_ = r_rect.size;
  // 這個angle最后被取
  this->angle_ = r_rect.angle;
  this->center_ = r_rect.center;
  if (cv::getVersionMinor() >= 5) {
    NewToOld(r_rect);
  }
  FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
  this->UpdateVertices();
}

CustomRect::CustomRect(const cv::Point2f &center, const cv::Size2f &size,
                       float angle) {
  this->center_ = center;
  this->size_ = size;
  this->angle_ = angle;
  FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
  this->UpdateVertices();
}

CustomRect::CustomRect(const cv::Point &center, const cv::Size &size,
                       float angle) {
  this->center_ = center;
  this->size_ = size;
  this->angle_ = angle;
  FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
  this->UpdateVertices();
}
// 用来更新四个定点参数，需要写出更新逻辑：

void CustomRect::UpdateVertices() {
  float r = pow((float(this->size_.width * this->size_.width) / 4.f) +
                    (float(this->size_.height * this->size_.height) / 4.f),
                0.5);
  // 这里转换为弧度制
  double a = double(this->angle_) * CV_PI / 180.f;
  double b = atan2(this->size_.height, this->size_.width);
  if (this->angle_ < 45) {
    this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a + b),
                            float(this->center_.y) + r * sin(a + b));
    this->tl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b),
                            float(this->center_.y) + r * sin(a - b));
    this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a + b),
                            float(this->center_.y) - r * sin(a + b));
    this->br_ = cv::Point2f(float(this->center_.x) + r * cos(a - b),
                            float(this->center_.y) - r * sin(a - b));
    return;
  }
  if (this->angle_ < 90) {
    this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a + b),
                            float(this->center_.y) + r * sin(a + b));
    this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b),
                            float(this->center_.y) + r * sin(a - b));
    this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b),
                            float(this->center_.y) - r * sin(a + b));
    this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a - b),
                            float(this->center_.y) - r * sin(a - b));
    return;
  }
  if (this->angle_ < 135) {
    this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a + b),
                            float(this->center_.y) + r * sin(a + b));
    this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b),
                            float(this->center_.y) + r * sin(a - b));
    this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b),
                            float(this->center_.y) - r * sin(a + b));
    this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a - b),
                            float(this->center_.y) - r * sin(a - b));
    return;
  }
  this->tr_ = cv::Point2f(float(this->center_.x) - r * cos(a + b),
                          float(this->center_.y) + r * sin(a + b));
  this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a - b),
                          float(this->center_.y) + r * sin(a - b));
  this->bl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b),
                          float(this->center_.y) - r * sin(a + b));
  this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a - b),
                          float(this->center_.y) - r * sin(a - b));
}

CustomRect CustomRect::minCustomRect(cv::InputArray _points, float angle) {
  cv::Mat hull;

  angle = -angle;

  while (angle > 0 || angle <= -90) {
    if (angle > 0) {
      angle -= 90;
    } else {
      angle += 90;
    }
  }

  cv::convexHull(_points, hull, true, true);

  if (hull.depth() != CV_32F) {
    cv::Mat temp;
    hull.convertTo(temp, CV_32F);
    hull = temp;
  }

  const cv::Point2f *h_points = hull.ptr<cv::Point2f>();

  angle = -angle;

  double maxx = 0;
  double maxy = 0;
  double minx = 0;
  double miny = 0;
  if (hull.size) {
    double x = cos(angle * CV_PI / 180.f) * h_points[0].x -
               sin(angle * CV_PI / 180.f) * h_points[0].y;
    double y = sin(angle * CV_PI / 180.f) * h_points[0].x +
               cos(angle * CV_PI / 180.f) * h_points[0].y;
    maxx = x;
    maxy = y;
    minx = maxx;
    miny = maxy;
  }

  for (int i = 0; i < hull.rows; i++) {
    double x = cos(angle * CV_PI / 180.f) * h_points[i].x -
               sin(angle * CV_PI / 180.f) * h_points[i].y;
    double y = sin(angle * CV_PI / 180.f) * h_points[i].x +
               cos(angle * CV_PI / 180.f) * h_points[i].y;
    maxx = maxx < x ? x : maxx;
    maxy = maxy < y ? y : maxy;
    minx = minx > x ? x : minx;
    miny = miny > y ? y : miny;
  }

  auto crect =
      CustomRect(cv::Point2d(cos(-angle * CV_PI / 180) * (maxx + minx) / 2 -
                                 sin(-angle * CV_PI / 180) * (maxy + miny) / 2,
                             sin(-angle * CV_PI / 180) * (maxx + minx) / 2 +
                                 cos(-angle * CV_PI / 180) * (maxy + miny) / 2),
                 cv::Size(maxx - minx, maxy - miny), -angle);
  return crect;
}

/**
 * @class RobotArmor
 */

RobotArmor::RobotArmor(const CustomRect &sticker_rect,
                       const RobotType &robot_type,
                       const std::vector<cv::Point2f> &image_point_list,
                       const int &believable) {
  robot_type_ = robot_type;
  sticker_rect_ = sticker_rect;
  image_point_list_ = image_point_list;
  WorldPointLists::GetRobotWorldPoints(robot_type, world_point_list_);
  believable_ = believable;
}
RobotArmor::RobotArmor(const CustomRect &sticker_rect,
                       const RobotType &robot_type,
                       const std::vector<cv::Point2f> &image_point_list,
                       const int &believable, int balance[3]) {
  robot_type_ = robot_type;
  sticker_rect_ = sticker_rect;
  image_point_list_ = image_point_list;
  WorldPointLists::GetRobotWorldPoints(robot_type, world_point_list_, balance);
  believable_ = believable;
}

RobotArmor::RobotArmor(const CustomRect &sticker_rect,
                       const DPRobotType &DProbot_type,
                       const vector<cv::Point2f> &image_point_list,
                       const Point2f &axis, const int &believable,
                       const float &conf) {
  DP_robot_type_ = DProbot_type;
  sticker_rect_ = sticker_rect;
  image_point_list_ = image_point_list;
  axis_ = axis;
  believable_ = believable;
  conf_ = conf;
}

void RobotArmor::SetWorldPoints(DPRobotType DProbot_type, int balance[3]) {
  WorldPointLists::GetDPRobotWorldPoints(DProbot_type, world_point_list_,
                                         balance);
}

void RobotArmor::SetWorldPoints(DPRobotType DProbot_type, bool isBalance) {
  WorldPointLists::GetDPRobotWorldPoints(DProbot_type, world_point_list_,
                                         isBalance);
}

/**
 * @class ResolvedArmor
 */

ResolvedArmor::ResolvedArmor(
    RobotType const &robot_type, CustomRect const &sticker_rect,
    cv::Rect const &armor_bounding_rect, Mat position_in_world,
    Vec3f const &euler_world_to_object, Polar3f const &polar_in_world,
    Mat pnp_tvec, Mat pnp_rvec, vector<Point2f> image_points,
    vector<Point3f> world_points, Point2f axis, int believable,
    cv::Vec3f TvecCameraInWorld_, float conf, float lightCenter_dis) {
  robot_type_ = robot_type;
  sticker_rect_ = sticker_rect;
  armor_bounding_rect_ = armor_bounding_rect;
  position_in_world_ = position_in_world.clone();
  position_in_world.convertTo(position_in_world, CV_32F);
  euler_world_to_object_ = euler_world_to_object;
  polar_in_world_ = polar_in_world;
  pnp_tvec_ = pnp_tvec.clone();
  pnp_rvec_ = pnp_rvec.clone();
  image_points_ = image_points;
  world_points_ = world_points;
  axis_ = axis;
  believable_ = believable;
  pnp_tvec_.convertTo(pnp_tvec_, CV_32F);
  pnp_rvec_.convertTo(pnp_rvec_, CV_32F);
  resolved = 1;
  TvecCameraInWorld = Mat(TvecCameraInWorld_);
  conf_ = conf;
  lightCenter_dis_ = lightCenter_dis;
}

ResolvedArmor::ResolvedArmor() {
  robot_type_ = TYPEUNKNOW;
  sticker_rect_ = CustomRect();
  position_in_world_ = Mat(3, 3, CV_32F, Scalar::all(0));
  euler_world_to_object_ = Vec3f(0, 0, 0);
  polar_in_world_ = Polar3f(0, 0, 0);
  pnp_rvec_ = Mat(3, 3, CV_32F, Scalar::all(0));
  pnp_tvec_ = Mat(3, 1, CV_32F, Scalar::all(0));
  image_points_ = vector<Point2f>();
  world_points_ = vector<Point3f>();
  axis_ = Point2f();
  int tag_ = -1;
  bool believable_ = 0;
  resolved = 0;
  conf_ = 0;
}

/**
 * @class Robot
 */

Robot::Robot() = default; /*NOLINT*/

Robot::Robot(vector<ResolvedArmor> const &resolved_armors,
             RobotType const &robot_type, uint64_t const &time_stamp) {
  robot_type_ = robot_type;
  resolved_armors_ = resolved_armors;
  time_stamp_ = time_stamp;
}
Robot::Robot(ResolvedArmor &resolved_armor, RobotType const &robot_type,
             uint64_t const &time_stamp) {
  robot_type_ = robot_type;
  time_stamp_ = time_stamp;
  resolved_armors_.push_back(resolved_armor);
  tvecCameraInWorld = resolved_armor.GetTvecCameraInWorld();
  axisInPixel = resolved_armor.Getaxis();
}

/**
 * @class BuffArmor
 */

BuffArmor::BuffArmor(CustomRect const &buff_rect) {
  buff_rect_ = buff_rect;
  empty_ = false;
  rotate_angle_ = 0;
  circle_point_ = cv::Point2f(0, 0);
  buff_type_ = None;
  is_r_ = 2;
  svm_flow_water_ = 2;
  kd_flow_water_ = 100;
  single_R_;
  single_T_;
}

/**
 * @class Buff2
 */
Buff2::Buff2() { empty_ = true; }

Buff2::Buff2(std::vector<BuffArmor> const &buff_armor,
             cv::Mat const &vec_rvec_object_to_camera,
             cv::Mat const &tvec_object_in_camera,
             //               cv::Mat const &pnp_rvec_object_to_camera,
             //               cv::Mat const &pnp_tvec_object_in_camera,
             cv::Mat const &R_rvec_object_in_camera,
             cv::Mat const &R_tvec_object_in_camera) {
  // TODO 命名
  empty_ = false;
  buff_armors_ = buff_armor;
  vec_rvec_object_to_camera_ = vec_rvec_object_to_camera;
  tvec_object_in_camera_ = tvec_object_in_camera;
  //        pnp_rvec_object_to_camera_     = pnp_rvec_object_to_camera;
  //        pnp_tvec_object_in_camera_     = pnp_tvec_object_in_camera;
  R_rvec_object_in_camera_ = R_rvec_object_in_camera;
  R_tvec_object_in_camera_ = R_tvec_object_in_camera;
  target_world_point = *tvec_object_in_camera.ptr<cv::Point3f>();
  target_world_polar = RectangularToPolar(target_world_point);
  CompleteBuff();
}

void Buff2::CompleteBuff() {
  int empty_nums = 0;
  for (int i = 0; i < 5; i++) {
    if (!buff_armors_[i].GetEmpty())
      continue;
    else {
      float rotate_angle_i = i * 2 * CV_PI / 5;
      cv::Point2f armor_point = buff_armors_[0].GetBuffRect().GetCenter2f() -
                                buff_armors_[0].GetCirclePoint();
      cv::Point3f armor_false_point3f =
          cv::Point3f(armor_point.x, armor_point.y, 0);
      cv::Vec3f rotate_vec = cv::Vec3f{0, 0, rotate_angle_i};
      cv::Mat rotate_mat = cv::Mat(rotate_vec);
      tdttoolkit::Rodrigues(rotate_mat, rotate_mat);
      rotate_mat = rotate_mat * cv::Mat(armor_false_point3f);
      armor_false_point3f = cv::Point3f(rotate_mat);
      armor_point = cv::Point2f(armor_false_point3f.x, armor_false_point3f.y);
      armor_point += buff_armors_[0].GetCirclePoint();
      tdttoolkit::CustomRect buff_armor_i = tdttoolkit::CustomRect(
          armor_point, buff_armors_[0].GetBuffRect().GetSize(), rotate_angle_i);
      buff_armors_[i] = BuffArmor(buff_armor_i);
      bool empty = true;
      buff_armors_[i].SetEmpty(empty);
      empty_nums += 1;
      final_index = i;
    }
  }
  if (empty_nums != 1) final_index = 0;
}

}  // namespace tdttoolkit
