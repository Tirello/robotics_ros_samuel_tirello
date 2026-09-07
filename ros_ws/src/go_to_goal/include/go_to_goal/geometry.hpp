// Copyright 2026 Samuel Tirello
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef GO_TO_GOAL__GEOMETRY_HPP_
#define GO_TO_GOAL__GEOMETRY_HPP_

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace go_to_goal::geometry
{

/// Normaliza um ângulo para (-pi, pi].

inline double wrap_to_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Limita `value` a [-|limit|, +|limit|].

inline double saturate(double value, double limit)
{
  const double bound = std::abs(limit);
  return std::clamp(value, -bound, bound);
}

/// Yaw do quaternion, assumindo movimento planar.

inline double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace go_to_goal::geometry

#endif  // GO_TO_GOAL__GEOMETRY_HPP_
