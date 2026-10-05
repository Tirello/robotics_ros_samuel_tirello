#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "custom_interfaces/msg/point2_d.hpp"
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

using namespace std::chrono_literals;

template<typename ServiceT, typename PrintResponse>
int call_service(
  const rclcpp::Node::SharedPtr & node,
  const std::string & service_name,
  const std::shared_ptr<typename ServiceT::Request> & request,
  PrintResponse print_response)
{
  auto client = node->create_client<ServiceT>(service_name);
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted while waiting for service");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for /%s...", service_name.c_str());
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("Service call failed");
  }
  print_response(future.get());
  return 0;
}

std::vector<double> parse_csv(const std::string & text)
{
  std::vector<double> values;
  if (text.empty()) {
    return values;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    values.push_back(std::stod(token));
  }
  return values;
}

std::vector<custom_interfaces::msg::Point2D> parse_points(const std::string & text)
{
  std::vector<custom_interfaces::msg::Point2D> points;
  if (text.empty()) {
    return points;
  }
  std::stringstream stream(text);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const auto separator = token.find(':');
    if (separator == std::string::npos) {
      throw std::invalid_argument("Each point must use x:y");
    }
    custom_interfaces::msg::Point2D point;
    point.x = std::stod(token.substr(0, separator));
    point.y = std::stod(token.substr(separator + 1));
    points.push_back(point);
  }
  return points;
}

void require_arguments(int argc, int expected, const std::string & usage)
{
  if (argc != expected) {
    throw std::invalid_argument("Usage: service_client " + usage);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sonho_service_client_cpp");
  try {
    if (argc < 2) {
      throw std::invalid_argument(
              "Usage: service_client <subtract|even|temperature|circle|reverse|dot|stats|"
              "regression|closest|ik> ...");
    }
    const std::string operation = argv[1];

    if (operation == "subtract") {
      require_arguments(argc, 4, "subtract A B");
      auto request = std::make_shared<custom_interfaces::srv::SubtractTwoInts::Request>();
      request->a = std::stoll(argv[2]);
      request->b = std::stoll(argv[3]);
      return call_service<custom_interfaces::srv::SubtractTwoInts>(
        node, "subtract_two_ints", request,
        [](const auto & response) {std::cout << response->difference << '\n';});
    }
    if (operation == "even") {
      require_arguments(argc, 3, "even NUMBER");
      auto request = std::make_shared<custom_interfaces::srv::IsEven::Request>();
      request->number = std::stoll(argv[2]);
      return call_service<custom_interfaces::srv::IsEven>(
        node, "is_even", request,
        [](const auto & response) {std::cout << std::boolalpha << response->is_even << '\n';});
    }
    if (operation == "temperature") {
      require_arguments(argc, 3, "temperature CELSIUS");
      auto request = std::make_shared<custom_interfaces::srv::ConvertTemperature::Request>();
      request->celsius = std::stod(argv[2]);
      return call_service<custom_interfaces::srv::ConvertTemperature>(
        node, "convert_temperature", request,
        [](const auto & response) {std::cout << response->fahrenheit << '\n';});
    }
    if (operation == "circle") {
      require_arguments(argc, 3, "circle RADIUS");
      auto request = std::make_shared<custom_interfaces::srv::GetCircleArea::Request>();
      request->radius = std::stod(argv[2]);
      return call_service<custom_interfaces::srv::GetCircleArea>(
        node, "get_circle_area", request,
        [](const auto & response) {std::cout << response->area << '\n';});
    }
    if (operation == "reverse") {
      require_arguments(argc, 3, "reverse TEXT");
      auto request = std::make_shared<custom_interfaces::srv::ReverseString::Request>();
      request->text = argv[2];
      return call_service<custom_interfaces::srv::ReverseString>(
        node, "reverse_string", request,
        [](const auto & response) {std::cout << response->reversed_text << '\n';});
    }
    if (operation == "dot") {
      require_arguments(argc, 8, "dot AX AY AZ BX BY BZ");
      auto request = std::make_shared<custom_interfaces::srv::DotProduct::Request>();
      request->vector_a.x = std::stod(argv[2]);
      request->vector_a.y = std::stod(argv[3]);
      request->vector_a.z = std::stod(argv[4]);
      request->vector_b.x = std::stod(argv[5]);
      request->vector_b.y = std::stod(argv[6]);
      request->vector_b.z = std::stod(argv[7]);
      return call_service<custom_interfaces::srv::DotProduct>(
        node, "dot_product", request,
        [](const auto & response) {std::cout << response->result << '\n';});
    }
    if (operation == "stats") {
      require_arguments(argc, 3, "stats CSV_VALUES (use an empty quoted string for empty input)");
      auto request = std::make_shared<custom_interfaces::srv::ArrayStatistics::Request>();
      request->values = parse_csv(argv[2]);
      return call_service<custom_interfaces::srv::ArrayStatistics>(
        node, "array_statistics", request,
        [](const auto & response) {
          std::cout << "min=" << response->minimum << " max=" << response->maximum <<
            " average=" << response->average << '\n';
        });
    }
    if (operation == "regression") {
      require_arguments(argc, 4, "regression X_CSV Y_CSV");
      auto request = std::make_shared<custom_interfaces::srv::LinearRegression::Request>();
      request->x = parse_csv(argv[2]);
      request->y = parse_csv(argv[3]);
      return call_service<custom_interfaces::srv::LinearRegression>(
        node, "linear_regression", request,
        [](const auto & response) {
          std::cout << "slope=" << response->slope << " intercept=" << response->intercept <<
            " r_squared=" << response->r_squared << '\n';
        });
    }
    if (operation == "closest") {
      require_arguments(argc, 5, "closest REF_X REF_Y 'X1:Y1,X2:Y2'");
      auto request = std::make_shared<custom_interfaces::srv::ClosestPoint::Request>();
      request->reference.x = std::stod(argv[2]);
      request->reference.y = std::stod(argv[3]);
      request->candidates = parse_points(argv[4]);
      return call_service<custom_interfaces::srv::ClosestPoint>(
        node, "closest_point", request,
        [](const auto & response) {
          std::cout << "closest=(" << response->closest.x << ',' << response->closest.y <<
            ") distance=" << response->distance << " index=" << response->index << '\n';
        });
    }
    if (operation == "ik") {
      require_arguments(argc, 6, "ik TARGET_X TARGET_Y LINK1 LINK2");
      auto request = std::make_shared<custom_interfaces::srv::InverseKinematics2Link::Request>();
      request->target_x = std::stod(argv[2]);
      request->target_y = std::stod(argv[3]);
      request->link1_length = std::stod(argv[4]);
      request->link2_length = std::stod(argv[5]);
      return call_service<custom_interfaces::srv::InverseKinematics2Link>(
        node, "inverse_kinematics_2link", request,
        [](const auto & response) {
          std::cout << std::boolalpha << "reachable=" << response->reachable <<
            " theta1=" << response->theta1 << " theta2=" << response->theta2 << '\n';
        });
    }
    throw std::invalid_argument("Unknown operation: " + operation);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
}
