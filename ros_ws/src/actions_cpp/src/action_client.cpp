#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "custom_interfaces/action/count_up.hpp"
#include "custom_interfaces/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

int run_count(const rclcpp::Node::SharedPtr & node, int32_t limit)
{
  using Action = custom_interfaces::action::CountUp;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  auto client = rclcpp_action::create_client<Action>(node, "count_up");
  if (!client->wait_for_action_server(5s)) {
    throw std::runtime_error("CountUp action server not available");
  }

  Action::Goal goal;
  goal.limit = limit;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
      std::cout << "Feedback: " << feedback->current_value << '\n';
    };

  auto goal_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("Failed to send CountUp goal");
  }
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    std::cout << "Goal rejected.\n";
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("Failed while waiting for CountUp result");
  }
  const auto wrapped = result_future.get();
  std::cout << std::boolalpha << "success=" << wrapped.result->success <<
    " final_value=" << wrapped.result->final_value <<
    " message=\"" << wrapped.result->message << "\"\n";
  return 0;
}

int run_timer(
  const rclcpp::Node::SharedPtr & node,
  double duration,
  double cancel_after)
{
  using Action = custom_interfaces::action::Timer;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  auto client = rclcpp_action::create_client<Action>(node, "timer");
  if (!client->wait_for_action_server(5s)) {
    throw std::runtime_error("Timer action server not available");
  }

  Action::Goal goal;
  goal.duration = duration;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
      std::cout << "Feedback: " << feedback->elapsed << " s\n";
    };

  auto goal_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("Failed to send Timer goal");
  }
  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    std::cout << "Goal rejected.\n";
    return 1;
  }

  rclcpp::TimerBase::SharedPtr cancel_timer;
  if (cancel_after >= 0.0) {
    const auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(cancel_after));
    cancel_timer = node->create_wall_timer(
      delay,
      [client, goal_handle, &cancel_timer]() {
        std::cout << "Cancellation requested.\n";
        client->async_cancel_goal(goal_handle);
        cancel_timer->cancel();
      });
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("Failed while waiting for Timer result");
  }
  const auto wrapped = result_future.get();
  std::cout << std::boolalpha << "completed=" << wrapped.result->completed <<
    " elapsed=" << wrapped.result->elapsed <<
    " message=\"" << wrapped.result->message << "\"\n";
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sonho_action_client_cpp");
  try {
    if (argc < 3) {
      throw std::invalid_argument(
              "Usage: action_client count LIMIT | action_client timer DURATION [CANCEL_AFTER]");
    }
    const std::string operation = argv[1];
    int result = 0;
    if (operation == "count" && argc == 3) {
      result = run_count(node, static_cast<int32_t>(std::stoi(argv[2])));
    } else if (operation == "timer" && (argc == 3 || argc == 4)) {
      const double cancel_after = argc == 4 ? std::stod(argv[3]) : -1.0;
      result = run_timer(node, std::stod(argv[2]), cancel_after);
    } else {
      throw std::invalid_argument("Invalid operation or argument count");
    }
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
}
