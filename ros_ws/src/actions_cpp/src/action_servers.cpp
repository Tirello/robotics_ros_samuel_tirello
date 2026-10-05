#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "custom_interfaces/action/count_up.hpp"
#include "custom_interfaces/action/timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class ActionServers : public rclcpp::Node
{
public:
  using CountUp = custom_interfaces::action::CountUp;
  using CountGoalHandle = rclcpp_action::ServerGoalHandle<CountUp>;
  using Timer = custom_interfaces::action::Timer;
  using TimerGoalHandle = rclcpp_action::ServerGoalHandle<Timer>;

  ActionServers() : Node("sonho_action_servers_cpp")
  {
    count_server_ = rclcpp_action::create_server<CountUp>(
      this,
      "count_up",
      std::bind(&ActionServers::handle_count_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServers::handle_count_cancel, this, std::placeholders::_1),
      std::bind(&ActionServers::handle_count_accepted, this, std::placeholders::_1));

    timer_server_ = rclcpp_action::create_server<Timer>(
      this,
      "timer",
      std::bind(&ActionServers::handle_timer_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServers::handle_timer_cancel, this, std::placeholders::_1),
      std::bind(&ActionServers::handle_timer_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CountUp and Timer action servers are ready (C++).");
  }

private:
  rclcpp_action::GoalResponse handle_count_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CountUp::Goal> goal)
  {
    if (goal->limit < 0) {
      RCLCPP_WARN(get_logger(), "Rejected CountUp goal with negative limit.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_count_cancel(
    const std::shared_ptr<CountGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_count_accepted(const std::shared_ptr<CountGoalHandle> goal_handle)
  {
    std::thread{std::bind(&ActionServers::execute_count, this, goal_handle)}.detach();
  }

  void execute_count(const std::shared_ptr<CountGoalHandle> goal_handle)
  {
    const int32_t limit = goal_handle->get_goal()->limit;
    auto feedback = std::make_shared<CountUp::Feedback>();
    auto result = std::make_shared<CountUp::Result>();
    int32_t last_value = 0;
    std::this_thread::sleep_for(50ms);

    for (int32_t value = 1; value <= limit; ++value) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->final_value = last_value;
        result->message = "Count canceled.";
        goal_handle->canceled(result);
        return;
      }
      last_value = value;
      feedback->current_value = value;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(250ms);
    }

    result->success = true;
    result->final_value = last_value;
    result->message = "Count completed successfully.";
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse handle_timer_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Timer::Goal> goal)
  {
    if (!std::isfinite(goal->duration) || goal->duration <= 0.0) {
      RCLCPP_WARN(get_logger(), "Rejected Timer goal with non-positive duration.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_timer_cancel(
    const std::shared_ptr<TimerGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_timer_accepted(const std::shared_ptr<TimerGoalHandle> goal_handle)
  {
    std::thread{std::bind(&ActionServers::execute_timer, this, goal_handle)}.detach();
  }

  void execute_timer(const std::shared_ptr<TimerGoalHandle> goal_handle)
  {
    const double requested_duration = goal_handle->get_goal()->duration;
    const auto start = std::chrono::steady_clock::now();
    auto feedback = std::make_shared<Timer::Feedback>();
    auto result = std::make_shared<Timer::Result>();
    double elapsed = 0.0;
    double next_feedback = std::min(1.0, requested_duration);

    while (elapsed < requested_duration) {
      if (goal_handle->is_canceling()) {
        result->completed = false;
        result->elapsed = elapsed;
        result->message = "Timer canceled before completion.";
        goal_handle->canceled(result);
        return;
      }
      std::this_thread::sleep_for(50ms);
      elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= next_feedback || elapsed >= requested_duration) {
        feedback->elapsed = std::min(elapsed, requested_duration);
        goal_handle->publish_feedback(feedback);
        next_feedback += 1.0;
      }
    }

    result->completed = true;
    result->elapsed = requested_duration;
    result->message = "Timer completed.";
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<CountUp>::SharedPtr count_server_;
  rclcpp_action::Server<Timer>::SharedPtr timer_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionServers>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
