// Copyright 2026 Samuel Tirello
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>

#include "go_to_goal/controller.hpp"

namespace
{

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

geometry_msgs::msg::Vector3 make_goal(double x, double y)
{
  geometry_msgs::msg::Vector3 goal;
  goal.x = x;
  goal.y = y;
  return goal;
}

constexpr double kTolerance = 1e-6;

}  // namespace

TEST(Geometry, WrapToPiRemovesFullTurns)
{
  using go_to_goal::geometry::wrap_to_pi;
  EXPECT_NEAR(wrap_to_pi(1.5 * M_PI), -0.5 * M_PI, kTolerance);
  EXPECT_NEAR(wrap_to_pi(-1.5 * M_PI), 0.5 * M_PI, kTolerance);
  EXPECT_NEAR(wrap_to_pi(2.0 * M_PI + 0.5), 0.5, kTolerance);
  EXPECT_NEAR(wrap_to_pi(0.5), 0.5, kTolerance);
  // +pi e -pi descrevem a mesma direção; o valor absoluto é o invariante.
  EXPECT_NEAR(std::abs(wrap_to_pi(3.0 * M_PI)), M_PI, kTolerance);
}

TEST(Geometry, SaturateIsSymmetricAndIgnoresLimitSign)
{
  using go_to_goal::geometry::saturate;
  EXPECT_DOUBLE_EQ(saturate(5.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(saturate(-5.0, 2.0), -2.0);
  EXPECT_DOUBLE_EQ(saturate(-5.0, -2.0), -2.0);
  EXPECT_DOUBLE_EQ(saturate(1.0, 2.0), 1.0);
}

TEST(Controller, StopsInsidePositionTolerance)
{
  const go_to_goal::ControllerGains gains;
  const auto command = go_to_goal::compute_velocity_command(
    make_pose(1.0, 1.0, 0.0), make_goal(1.01, 1.0), gains);

  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(Controller, DrivesStraightWhenAligned)
{
  go_to_goal::ControllerGains gains;
  gains.linear = 0.2;

  const auto command = go_to_goal::compute_velocity_command(
    make_pose(0.0, 0.0, 0.0), make_goal(3.0, 0.0), gains);

  EXPECT_NEAR(command.linear.x, 0.6, kTolerance);
  EXPECT_NEAR(command.angular.z, 0.0, kTolerance);
}

TEST(Controller, TurnsInPlaceWhenGoalIsPerpendicular)
{
  const go_to_goal::ControllerGains gains;
  const auto command = go_to_goal::compute_velocity_command(
    make_pose(0.0, 0.0, 0.0), make_goal(0.0, 2.0), gains);

  EXPECT_NEAR(command.linear.x, 0.0, kTolerance);
  EXPECT_DOUBLE_EQ(command.angular.z, gains.max_angular_velocity);
}

TEST(Controller, ReversesWhenGoalIsBehind)
{
  go_to_goal::ControllerGains gains;
  gains.angular = 0.0;  // isola o eixo linear

  const auto command = go_to_goal::compute_velocity_command(
    make_pose(0.0, 0.0, 0.0), make_goal(-1.0, 0.0), gains);

  EXPECT_LT(command.linear.x, 0.0);
}

TEST(Controller, RespectsVelocityLimits)
{
  go_to_goal::ControllerGains gains;
  gains.max_linear_velocity = 0.3;
  gains.max_angular_velocity = 0.4;

  const auto command = go_to_goal::compute_velocity_command(
    make_pose(0.0, 0.0, 0.0), make_goal(50.0, 20.0), gains);

  EXPECT_LE(std::abs(command.linear.x), gains.max_linear_velocity);
  EXPECT_LE(std::abs(command.angular.z), gains.max_angular_velocity);
}
