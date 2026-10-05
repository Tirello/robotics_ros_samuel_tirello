"""Ten ROS 2 service servers required by Lista 7 - Servicos."""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from custom_interfaces.srv import ArrayStatistics
from custom_interfaces.srv import ClosestPoint
from custom_interfaces.srv import ConvertTemperature
from custom_interfaces.srv import DotProduct
from custom_interfaces.srv import GetCircleArea
from custom_interfaces.srv import InverseKinematics2Link
from custom_interfaces.srv import IsEven
from custom_interfaces.srv import LinearRegression
from custom_interfaces.srv import ReverseString
from custom_interfaces.srv import SubtractTwoInts
from rclpy.node import Node


class ServiceServers(Node):
    """Expose the ten independent exercise services in one process."""

    EPSILON = 1.0e-12

    def __init__(self) -> None:
        super().__init__("sonho_service_servers_py")
        self._services = [
            self.create_service(SubtractTwoInts, "subtract_two_ints", self.subtract),
            self.create_service(IsEven, "is_even", self.is_even),
            self.create_service(
                ConvertTemperature, "convert_temperature", self.convert_temperature
            ),
            self.create_service(GetCircleArea, "get_circle_area", self.circle_area),
            self.create_service(ReverseString, "reverse_string", self.reverse_string),
            self.create_service(DotProduct, "dot_product", self.dot_product),
            self.create_service(
                ArrayStatistics, "array_statistics", self.array_statistics
            ),
            self.create_service(
                LinearRegression, "linear_regression", self.linear_regression
            ),
            self.create_service(ClosestPoint, "closest_point", self.closest_point),
            self.create_service(
                InverseKinematics2Link,
                "inverse_kinematics_2link",
                self.inverse_kinematics,
            ),
        ]
        self.get_logger().info("Ten SONHO service servers are ready (Python).")

    @staticmethod
    def subtract(request, response):
        response.difference = request.a - request.b
        return response

    @staticmethod
    def is_even(request, response):
        response.is_even = request.number % 2 == 0
        return response

    @staticmethod
    def convert_temperature(request, response):
        response.fahrenheit = request.celsius * 9.0 / 5.0 + 32.0
        return response

    @staticmethod
    def circle_area(request, response):
        response.area = -1.0 if request.radius < 0.0 else math.pi * request.radius**2
        return response

    @staticmethod
    def reverse_string(request, response):
        response.reversed_text = request.text[::-1]
        return response

    @staticmethod
    def dot_product(request, response):
        response.result = (
            request.vector_a.x * request.vector_b.x
            + request.vector_a.y * request.vector_b.y
            + request.vector_a.z * request.vector_b.z
        )
        return response

    @staticmethod
    def array_statistics(request, response):
        if not request.values:
            response.minimum = 0.0
            response.maximum = 0.0
            response.average = 0.0
            return response
        response.minimum = min(request.values)
        response.maximum = max(request.values)
        response.average = sum(request.values) / len(request.values)
        return response

    @classmethod
    def linear_regression(cls, request, response):
        if not request.x or len(request.x) != len(request.y):
            return cls._zero_regression(response)

        count = float(len(request.x))
        sum_x = sum(request.x)
        sum_y = sum(request.y)
        sum_xx = sum(value * value for value in request.x)
        sum_xy = sum(x * y for x, y in zip(request.x, request.y))
        slope_denominator = count * sum_xx - sum_x * sum_x
        if abs(slope_denominator) <= cls.EPSILON:
            return cls._zero_regression(response)

        response.slope = (count * sum_xy - sum_x * sum_y) / slope_denominator
        response.intercept = (sum_y - response.slope * sum_x) / count
        mean_y = sum_y / count
        residual_sum = sum(
            (y - (response.slope * x + response.intercept)) ** 2
            for x, y in zip(request.x, request.y)
        )
        total_sum = sum((y - mean_y) ** 2 for y in request.y)
        if total_sum <= cls.EPSILON:
            response.r_squared = 1.0 if residual_sum <= cls.EPSILON else 0.0
        else:
            response.r_squared = 1.0 - residual_sum / total_sum
        return response

    @staticmethod
    def _zero_regression(response):
        response.slope = 0.0
        response.intercept = 0.0
        response.r_squared = 0.0
        return response

    @staticmethod
    def closest_point(request, response):
        if not request.candidates:
            response.closest.x = 0.0
            response.closest.y = 0.0
            response.distance = -1.0
            response.index = -1
            return response

        distances = [
            math.hypot(
                candidate.x - request.reference.x,
                candidate.y - request.reference.y,
            )
            for candidate in request.candidates
        ]
        response.index = min(range(len(distances)), key=distances.__getitem__)
        response.closest.x = request.candidates[response.index].x
        response.closest.y = request.candidates[response.index].y
        response.distance = distances[response.index]
        return response

    @classmethod
    def inverse_kinematics(cls, request, response):
        l1 = request.link1_length
        l2 = request.link2_length
        radius = math.hypot(request.target_x, request.target_y)
        if (
            l1 <= 0.0
            or l2 <= 0.0
            or radius < abs(l1 - l2) - cls.EPSILON
            or radius > l1 + l2 + cls.EPSILON
        ):
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.reachable = False
            return response

        raw_cos_theta2 = (radius**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        cos_theta2 = max(-1.0, min(1.0, raw_cos_theta2))
        response.theta2 = math.acos(cos_theta2)
        k1 = l1 + l2 * math.cos(response.theta2)
        k2 = l2 * math.sin(response.theta2)
        response.theta1 = math.atan2(request.target_y, request.target_x) - math.atan2(
            k2, k1
        )
        response.reachable = True
        return response


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServiceServers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
