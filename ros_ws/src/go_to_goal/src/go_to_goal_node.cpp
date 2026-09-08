// Copyright 2026 Samuel Tirello
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "go_to_goal/controller.hpp"

namespace go_to_goal
{
namespace
{

using namespace std::chrono_literals;
using std::placeholders::_1;

// Fixo, e não um parâmetro: a taxa de publicação é contrato de interface com
// quem consome /cmd_vel, não ajuste de sintonia.
constexpr auto kControlPeriod = 100ms;

rcl_interfaces::msg::ParameterDescriptor bounded(
  const std::string & description, double minimum, double maximum)
{
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = minimum;
  range.to_value = maximum;

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.floating_point_range = {range};
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor described(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

}  // namespace

/// Conduz o robô até um objetivo publicando velocidade a 10 Hz.
///
/// Assina /goal e /robot_position; publica /cmd_vel. Sem objetivo, publica
/// velocidade nula: silêncio não equivale a comando de parada, porque a maioria
/// das bases mantém a última velocidade até um timeout.
class GoToGoalNode : public rclcpp::Node
{
public:
  GoToGoalNode()
  : Node("go_to_goal_node")
  {
    declare_parameters();

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    goal_subscription_ = create_subscription<geometry_msgs::msg::Vector3>(
      "goal", 10, std::bind(&GoToGoalNode::on_goal, this, _1));

    robot_position_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "robot_position", 10, std::bind(&GoToGoalNode::on_robot_position, this, _1));

    control_timer_ = create_wall_timer(
      kControlPeriod, std::bind(&GoToGoalNode::publish_command, this));

    RCLCPP_INFO(get_logger(), "go_to_goal_node ativo: publicando /cmd_vel a 10 Hz.");
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>(
      "kp_linear", 0.5, bounded("Ganho proporcional da velocidade linear.", 0.0, 10.0));
    declare_parameter<double>(
      "kp_angular", 1.5, bounded("Ganho proporcional da velocidade angular.", 0.0, 10.0));
    declare_parameter<double>(
      "max_linear_velocity", 1.0, bounded("Velocidade linear máxima [m/s].", 0.0, 5.0));
    declare_parameter<double>(
      "max_angular_velocity", 2.0, bounded("Velocidade angular máxima [rad/s].", 0.0, 10.0));
    declare_parameter<double>(
      "position_tolerance", 0.05, bounded("Raio de chegada [m].", 0.0, 1.0));
    declare_parameter<std::string>(
      "frame_id", "base_link", described("Frame do comando de velocidade."));
  }

  ControllerGains current_gains() const
  {
    return ControllerGains{
      get_parameter("kp_linear").as_double(),
      get_parameter("kp_angular").as_double(),
      get_parameter("max_linear_velocity").as_double(),
      get_parameter("max_angular_velocity").as_double(),
      get_parameter("position_tolerance").as_double()};
  }

  void on_goal(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    RCLCPP_INFO(get_logger(), "Objetivo recebido: (%.2f, %.2f)", goal_.x, goal_.y);
  }

  void on_robot_position(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    robot_pose_ = *msg;
    has_robot_position_ = true;
  }

  void publish_command()
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = get_parameter("frame_id").as_string();

    if (has_goal_ && has_robot_position_) {
      command.twist = compute_velocity_command(robot_pose_, goal_, current_gains());
    }

    cmd_vel_publisher_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_position_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  geometry_msgs::msg::Vector3 goal_;
  geometry_msgs::msg::Pose robot_pose_;

  // Um Vector3 zerado é um objetivo válido (a origem), então a chegada de cada
  // entrada precisa ser rastreada explicitamente.
  bool has_goal_{false};
  bool has_robot_position_{false};
};

}  // namespace go_to_goal

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go_to_goal::GoToGoalNode>());
  rclcpp::shutdown();
  return 0;
}
