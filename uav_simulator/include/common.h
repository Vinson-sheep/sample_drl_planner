// #pragma once
#ifndef COMMON_H
#define COMMON_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

// tool function
double Distance(const geometry_msgs::Point &lhs,
                const geometry_msgs::Point &rhs);
double Distance(const geometry_msgs::Pose &lhs, const geometry_msgs::Pose &rhs);
bool LineCircleShortestCrossPoint(const geometry_msgs::Point &center,
                                  const double radius,
                                  geometry_msgs::Point &cur_pos,
                                  const geometry_msgs::Point &next_pos,
                                  geometry_msgs::Point &cross_point);
double PointLineDistance(const geometry_msgs::Point &point, const double line_k,
                         const double line_b);
bool Translate(const geometry_msgs::Point &tr_point,
               geometry_msgs::Point &point);
bool Rotate(const double theta, geometry_msgs::Point &point);
bool LineParam(const geometry_msgs::Point &point_a,
               const geometry_msgs::Point &point_b, double &line_k,
               double &line_b);
bool HomoDirect(const geometry_msgs::Point &line_a_1,
                const geometry_msgs::Point &line_a_2,
                const geometry_msgs::Point &line_b_1,
                const geometry_msgs::Point &line_b_2);

#endif /* COMMON_H */