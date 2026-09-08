// Copyright 2026 Samuel Tirello
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef GO_TO_GOAL__CONTROLLER_HPP_
#define GO_TO_GOAL__CONTROLLER_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "go_to_goal/geometry.hpp"

namespace go_to_goal
{

struct ControllerGains
{
  double linear{0.5};
  double angular{1.5};
  double max_linear_velocity{1.0};
  double max_angular_velocity{2.0};
  double position_tolerance{0.05};
};

/// Comando de velocidade que conduz `pose` até `goal`, para um modelo uniciclo.
///
/// Dentro da tolerância de posição devolve velocidade nula. A orientação final
/// não é controlada: o alvo é um ponto, sem yaw de destino.

inline geometry_msgs::msg::Twist compute_velocity_command(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Vector3 & goal,
  const ControllerGains & gains)
{
  geometry_msgs::msg::Twist command;

  const double dx = goal.x - pose.position.x;
  const double dy = goal.y - pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= gains.position_tolerance) {
    return command;
  }

  const double heading_error = geometry::wrap_to_pi(
    std::atan2(dy, dx) - geometry::yaw_from_quaternion(pose.orientation));

    // O cosseno acopla o avanço ao alinhamento: o robô gira parado quando o alvo
    // está a 90 graus e dá ré quando o alvo está atrás, em vez de descrever arcos.

  command.linear.x = geometry::saturate(
    gains.linear * distance * std::cos(heading_error), gains.max_linear_velocity);
  command.angular.z = geometry::saturate(
    gains.angular * heading_error, gains.max_angular_velocity);

  return command;
}

}  // namespace go_to_goal

#endif  // GO_TO_GOAL__CONTROLLER_HPP_
