#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "custom_interfaces/srv/array_statistics.hpp"
#include "custom_interfaces/srv/closest_point.hpp"
#include "custom_interfaces/srv/convert_temperature.hpp"
#include "custom_interfaces/srv/dot_product.hpp"
#include "custom_interfaces/srv/get_circle_area.hpp"
#include "custom_interfaces/srv/inverse_kinematics2_link.hpp"
#include "custom_interfaces/srv/is_even.hpp"
#include "custom_interfaces/srv/linear_regression.hpp"
#include "custom_interfaces/srv/reverse_string.hpp"
#include "custom_interfaces/srv/subtract_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class ServiceServers : public rclcpp::Node
{
public:
  ServiceServers() : Node("sonho_service_servers_cpp")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    subtract_service_ = create_service<custom_interfaces::srv::SubtractTwoInts>(
      "subtract_two_ints", std::bind(&ServiceServers::subtract, this, _1, _2));
    is_even_service_ = create_service<custom_interfaces::srv::IsEven>(
      "is_even", std::bind(&ServiceServers::is_even, this, _1, _2));
    temperature_service_ = create_service<custom_interfaces::srv::ConvertTemperature>(
      "convert_temperature", std::bind(&ServiceServers::convert_temperature, this, _1, _2));
    circle_service_ = create_service<custom_interfaces::srv::GetCircleArea>(
      "get_circle_area", std::bind(&ServiceServers::circle_area, this, _1, _2));
    reverse_service_ = create_service<custom_interfaces::srv::ReverseString>(
      "reverse_string", std::bind(&ServiceServers::reverse_string, this, _1, _2));
    dot_service_ = create_service<custom_interfaces::srv::DotProduct>(
      "dot_product", std::bind(&ServiceServers::dot_product, this, _1, _2));
    statistics_service_ = create_service<custom_interfaces::srv::ArrayStatistics>(
      "array_statistics", std::bind(&ServiceServers::array_statistics, this, _1, _2));
    regression_service_ = create_service<custom_interfaces::srv::LinearRegression>(
      "linear_regression", std::bind(&ServiceServers::linear_regression, this, _1, _2));
    closest_service_ = create_service<custom_interfaces::srv::ClosestPoint>(
      "closest_point", std::bind(&ServiceServers::closest_point, this, _1, _2));
    ik_service_ = create_service<custom_interfaces::srv::InverseKinematics2Link>(
      "inverse_kinematics_2link", std::bind(&ServiceServers::inverse_kinematics, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Ten SONHO service servers are ready (C++).");
  }

private:
  static constexpr double kEpsilon = 1.0e-12;

  void subtract(
    const custom_interfaces::srv::SubtractTwoInts::Request::SharedPtr request,
    custom_interfaces::srv::SubtractTwoInts::Response::SharedPtr response)
  {
    response->difference = request->a - request->b;
  }

  void is_even(
    const custom_interfaces::srv::IsEven::Request::SharedPtr request,
    custom_interfaces::srv::IsEven::Response::SharedPtr response)
  {
    response->is_even = request->number % 2 == 0;
  }

  void convert_temperature(
    const custom_interfaces::srv::ConvertTemperature::Request::SharedPtr request,
    custom_interfaces::srv::ConvertTemperature::Response::SharedPtr response)
  {
    response->fahrenheit = request->celsius * 9.0 / 5.0 + 32.0;
  }

  void circle_area(
    const custom_interfaces::srv::GetCircleArea::Request::SharedPtr request,
    custom_interfaces::srv::GetCircleArea::Response::SharedPtr response)
  {
    if (request->radius < 0.0) {
      response->area = -1.0;
      return;
    }
    const double pi = std::acos(-1.0);
    response->area = pi * request->radius * request->radius;
  }

  void reverse_string(
    const custom_interfaces::srv::ReverseString::Request::SharedPtr request,
    custom_interfaces::srv::ReverseString::Response::SharedPtr response)
  {
    response->reversed_text = std::string(request->text.rbegin(), request->text.rend());
  }

  void dot_product(
    const custom_interfaces::srv::DotProduct::Request::SharedPtr request,
    custom_interfaces::srv::DotProduct::Response::SharedPtr response)
  {
    response->result =
      request->vector_a.x * request->vector_b.x +
      request->vector_a.y * request->vector_b.y +
      request->vector_a.z * request->vector_b.z;
  }

  void array_statistics(
    const custom_interfaces::srv::ArrayStatistics::Request::SharedPtr request,
    custom_interfaces::srv::ArrayStatistics::Response::SharedPtr response)
  {
    if (request->values.empty()) {
      response->minimum = 0.0;
      response->maximum = 0.0;
      response->average = 0.0;
      return;
    }

    const auto bounds = std::minmax_element(request->values.begin(), request->values.end());
    response->minimum = *bounds.first;
    response->maximum = *bounds.second;
    response->average =
      std::accumulate(request->values.begin(), request->values.end(), 0.0) /
      static_cast<double>(request->values.size());
  }

  void linear_regression(
    const custom_interfaces::srv::LinearRegression::Request::SharedPtr request,
    custom_interfaces::srv::LinearRegression::Response::SharedPtr response)
  {
    const std::size_t n = request->x.size();
    if (n == 0U || n != request->y.size()) {
      zero_regression(response);
      return;
    }

    const double sum_x = std::accumulate(request->x.begin(), request->x.end(), 0.0);
    const double sum_y = std::accumulate(request->y.begin(), request->y.end(), 0.0);
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      sum_xx += request->x[i] * request->x[i];
      sum_xy += request->x[i] * request->y[i];
    }

    const double count = static_cast<double>(n);
    const double slope_denominator = count * sum_xx - sum_x * sum_x;
    if (std::abs(slope_denominator) <= kEpsilon) {
      zero_regression(response);
      return;
    }

    response->slope = (count * sum_xy - sum_x * sum_y) / slope_denominator;
    response->intercept = (sum_y - response->slope * sum_x) / count;

    const double mean_y = sum_y / count;
    double residual_sum = 0.0;
    double total_sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double prediction = response->slope * request->x[i] + response->intercept;
      residual_sum += std::pow(request->y[i] - prediction, 2.0);
      total_sum += std::pow(request->y[i] - mean_y, 2.0);
    }

    response->r_squared = total_sum <= kEpsilon ?
      (residual_sum <= kEpsilon ? 1.0 : 0.0) : 1.0 - residual_sum / total_sum;
  }

  static void zero_regression(
    const custom_interfaces::srv::LinearRegression::Response::SharedPtr & response)
  {
    response->slope = 0.0;
    response->intercept = 0.0;
    response->r_squared = 0.0;
  }

  void closest_point(
    const custom_interfaces::srv::ClosestPoint::Request::SharedPtr request,
    custom_interfaces::srv::ClosestPoint::Response::SharedPtr response)
  {
    if (request->candidates.empty()) {
      response->closest.x = 0.0;
      response->closest.y = 0.0;
      response->distance = -1.0;
      response->index = -1;
      return;
    }

    std::size_t best_index = 0U;
    double best_distance = std::hypot(
      request->candidates[0].x - request->reference.x,
      request->candidates[0].y - request->reference.y);
    for (std::size_t i = 1U; i < request->candidates.size(); ++i) {
      const double distance = std::hypot(
        request->candidates[i].x - request->reference.x,
        request->candidates[i].y - request->reference.y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }

    response->closest = request->candidates[best_index];
    response->distance = best_distance;
    response->index = static_cast<int64_t>(best_index);
  }

  void inverse_kinematics(
    const custom_interfaces::srv::InverseKinematics2Link::Request::SharedPtr request,
    custom_interfaces::srv::InverseKinematics2Link::Response::SharedPtr response)
  {
    const double l1 = request->link1_length;
    const double l2 = request->link2_length;
    const double radius = std::hypot(request->target_x, request->target_y);
    if (l1 <= 0.0 || l2 <= 0.0 ||
      radius < std::abs(l1 - l2) - kEpsilon || radius > l1 + l2 + kEpsilon)
    {
      response->theta1 = 0.0;
      response->theta2 = 0.0;
      response->reachable = false;
      return;
    }

    const double raw_cos_theta2 =
      (radius * radius - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    const double cos_theta2 = std::clamp(raw_cos_theta2, -1.0, 1.0);
    response->theta2 = std::acos(cos_theta2);
    const double k1 = l1 + l2 * std::cos(response->theta2);
    const double k2 = l2 * std::sin(response->theta2);
    response->theta1 =
      std::atan2(request->target_y, request->target_x) - std::atan2(k2, k1);
    response->reachable = true;
  }

  rclcpp::Service<custom_interfaces::srv::SubtractTwoInts>::SharedPtr subtract_service_;
  rclcpp::Service<custom_interfaces::srv::IsEven>::SharedPtr is_even_service_;
  rclcpp::Service<custom_interfaces::srv::ConvertTemperature>::SharedPtr temperature_service_;
  rclcpp::Service<custom_interfaces::srv::GetCircleArea>::SharedPtr circle_service_;
  rclcpp::Service<custom_interfaces::srv::ReverseString>::SharedPtr reverse_service_;
  rclcpp::Service<custom_interfaces::srv::DotProduct>::SharedPtr dot_service_;
  rclcpp::Service<custom_interfaces::srv::ArrayStatistics>::SharedPtr statistics_service_;
  rclcpp::Service<custom_interfaces::srv::LinearRegression>::SharedPtr regression_service_;
  rclcpp::Service<custom_interfaces::srv::ClosestPoint>::SharedPtr closest_service_;
  rclcpp::Service<custom_interfaces::srv::InverseKinematics2Link>::SharedPtr ik_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServers>());
  rclcpp::shutdown();
  return 0;
}
