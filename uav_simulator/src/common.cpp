#include "common.h"

// tool function
double Distance(const geometry_msgs::Point &lhs,
                           const geometry_msgs::Point &rhs) {
  //
  return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
              (lhs.y - rhs.y) * (lhs.y - rhs.y));
}
double Distance(const geometry_msgs::Pose &lhs,
                           const geometry_msgs::Pose &rhs) {
  //
  return Distance(lhs.position, rhs.position);
}
bool LineCircleShortestCrossPoint(
    const geometry_msgs::Point &center, const double radius,
    geometry_msgs::Point &cur_pos, const geometry_msgs::Point &next_pos,
    geometry_msgs::Point &cross_point) {
  //
  double _line_k, _line_b;
  LineParam(cur_pos, next_pos, _line_k, _line_b);
  double _dist = PointLineDistance(center, _line_k, _line_b);
  //
  if (_dist > radius + std::numeric_limits<double>::epsilon()) {
    //
    return false;
  } else {
    //
    double _A = 1 + _line_k * _line_k;
    double _B = 2 * _line_k * (_line_b - center.y) - 2 * center.x;
    double _C = pow(center.x, 2) + pow(_line_b - center.y, 2) - pow(radius, 2);
    if (fabs(_dist - radius) < std::numeric_limits<double>::epsilon()) {
      cross_point.x = -_B / (2 * _A);
      cross_point.y = _line_k * cross_point.x + _line_b;
    } else {
      double _x1, _x2, _y1, _y2;
      double _D = sqrt(_B * _B - 4 * _A * _C);
      _x1 = (-_B + _D) / (2 * _A);
      _y1 = _line_k * _x1 + _line_b;
      _x2 = (-_B - _D) / (2 * _A);
      _y2 = _line_k * _x2 + _line_b;
      geometry_msgs::Point _cross_point_1;
      _cross_point_1.x = _x1;
      _cross_point_1.y = _y1;
      geometry_msgs::Point _cross_point_2;
      _cross_point_2.x = _x2;
      _cross_point_2.y = _y2;
      double _dist_1 = Distance(_cross_point_1, cur_pos);
      double _dist_2 = Distance(_cross_point_2, cur_pos);
      if (_dist_1 < _dist_2) {
        cross_point.x = _x1;
        cross_point.y = _y1;
      } else {
        cross_point.x = _x2;
        cross_point.y = _y2;
      }
    }
    // judge if cross point is in the right direction
    if (!HomoDirect(cur_pos, cross_point, cur_pos, next_pos)) {
      return false;
    }
  }
  return true;
}
double PointLineDistance(const geometry_msgs::Point &point,
                                    const double line_k, const double line_b) {
  //
  return fabs(line_k * point.x - point.y + line_b) /
         sqrt(1.0 + line_k * line_k);
}
bool Translate(const geometry_msgs::Point &tr_point,
                          geometry_msgs::Point &point) {
  //
  point.x += tr_point.x;
  point.y += tr_point.y;
  return true;
}
bool Rotate(const double theta, geometry_msgs::Point &point) {
  //
  double _dist = sqrt(point.x * point.x + point.y + point.y);
  double _theta_ori = atan2(point.y, point.x);
  _theta_ori += theta;
  point.x = _dist * cos(_theta_ori);
  point.y = _dist * sin(_theta_ori);
  return true;
}

bool LineParam(const geometry_msgs::Point &point_a,
                          const geometry_msgs::Point &point_b, double &line_k,
                          double &line_b) {
  //
  double _dx = point_a.x - point_b.x;
  double _dy = point_a.y - point_b.y;
  if (fabs(_dx) < 10e-7) {  // avoid zero divide error
    line_k = 10e17;
  } else {
    line_k = std::max(-10e17, std::min(10e17, _dy / _dx));
  }
  line_b = point_a.y - line_k * point_a.x;
  return true;
}

bool HomoDirect(const geometry_msgs::Point &line_a_1,
                           const geometry_msgs::Point &line_a_2,
                           const geometry_msgs::Point &line_b_1,
                           const geometry_msgs::Point &line_b_2) {
  //
  double _dx_a = line_a_1.x - line_a_2.x;
  double _dy_a = line_a_1.y - line_a_2.y;
  double _dx_b = line_b_1.x - line_b_2.x;
  double _dy_b = line_b_1.y - line_b_2.y;
  return fabs(std::atan2(_dy_a, _dx_a) - std::atan2(_dy_b, _dx_b)) < 10e-5;
}
void Interpolate(nav_msgs::Path &path, const double delta_d) {
  // 
  int32_t _n = path.poses.size();
  nav_msgs::Path _path_t;
  _path_t.header = path.header;
  _path_t.poses.push_back(path.poses.front());
  for (int32_t i = 1; i< _n; i++) {
    double _dist = Distance(path.poses[i-1].pose, path.poses[i].pose);
    double _dist_count = delta_d;
    double _theta = atan2(
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y,
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
    while (_dist_count < _dist) {
      geometry_msgs::Point _point_t;
      _point_t.x = _dist_count;
      _point_t.y = 0;
      _point_t.z = path.poses[i-1].pose.position.z;
      Rotate(_theta, _point_t);
      Translate(path.poses[i-1].pose.position, _point_t);
      geometry_msgs::PoseStamped _pst;
      _pst.pose.orientation = path.poses[i-1].pose.orientation;
      _pst.pose.position = _point_t;
      _path_t.poses.push_back(_pst);
      _dist_count += delta_d;
    }
    _path_t.poses.push_back(path.poses[i]);
  }
  path = _path_t;
}