"""Command-line Python client for all ten service exercises."""

from __future__ import annotations

import argparse
from typing import Sequence

import rclpy
from custom_interfaces.msg import Point2D
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


def csv_values(text: str) -> list[float]:
    return [] if text == "" else [float(value) for value in text.split(",")]


def points(text: str) -> list[Point2D]:
    result: list[Point2D] = []
    if text == "":
        return result
    for item in text.split(","):
        x_text, y_text = item.split(":", maxsplit=1)
        result.append(Point2D(x=float(x_text), y=float(y_text)))
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)

    subtract = commands.add_parser("subtract")
    subtract.add_argument("a", type=int)
    subtract.add_argument("b", type=int)

    even = commands.add_parser("even")
    even.add_argument("number", type=int)

    temperature = commands.add_parser("temperature")
    temperature.add_argument("celsius", type=float)

    circle = commands.add_parser("circle")
    circle.add_argument("radius", type=float)

    reverse = commands.add_parser("reverse")
    reverse.add_argument("text")

    dot = commands.add_parser("dot")
    dot.add_argument("values", nargs=6, type=float, metavar=("AX", "AY", "AZ", "BX", "BY", "BZ"))

    stats = commands.add_parser("stats")
    stats.add_argument("values", help="Comma-separated values; use an empty quoted string for empty input")

    regression = commands.add_parser("regression")
    regression.add_argument("x", help="Comma-separated x values")
    regression.add_argument("y", help="Comma-separated y values")

    closest = commands.add_parser("closest")
    closest.add_argument("reference_x", type=float)
    closest.add_argument("reference_y", type=float)
    closest.add_argument("candidates", help="Comma-separated x:y points")

    ik = commands.add_parser("ik")
    ik.add_argument("target_x", type=float)
    ik.add_argument("target_y", type=float)
    ik.add_argument("link1", type=float)
    ik.add_argument("link2", type=float)
    return parser


def prepare_call(arguments):
    operation = arguments.operation
    if operation == "subtract":
        return SubtractTwoInts, "subtract_two_ints", SubtractTwoInts.Request(a=arguments.a, b=arguments.b)
    if operation == "even":
        return IsEven, "is_even", IsEven.Request(number=arguments.number)
    if operation == "temperature":
        return ConvertTemperature, "convert_temperature", ConvertTemperature.Request(celsius=arguments.celsius)
    if operation == "circle":
        return GetCircleArea, "get_circle_area", GetCircleArea.Request(radius=arguments.radius)
    if operation == "reverse":
        return ReverseString, "reverse_string", ReverseString.Request(text=arguments.text)
    if operation == "dot":
        ax, ay, az, bx, by, bz = arguments.values
        request = DotProduct.Request()
        request.vector_a.x, request.vector_a.y, request.vector_a.z = ax, ay, az
        request.vector_b.x, request.vector_b.y, request.vector_b.z = bx, by, bz
        return DotProduct, "dot_product", request
    if operation == "stats":
        return ArrayStatistics, "array_statistics", ArrayStatistics.Request(values=csv_values(arguments.values))
    if operation == "regression":
        return LinearRegression, "linear_regression", LinearRegression.Request(
            x=csv_values(arguments.x), y=csv_values(arguments.y)
        )
    if operation == "closest":
        request = ClosestPoint.Request()
        request.reference = Point2D(x=arguments.reference_x, y=arguments.reference_y)
        request.candidates = points(arguments.candidates)
        return ClosestPoint, "closest_point", request
    if operation == "ik":
        return InverseKinematics2Link, "inverse_kinematics_2link", InverseKinematics2Link.Request(
            target_x=arguments.target_x,
            target_y=arguments.target_y,
            link1_length=arguments.link1,
            link2_length=arguments.link2,
        )
    raise ValueError(f"Unknown operation: {operation}")


def main(args: Sequence[str] | None = None) -> None:
    arguments = build_parser().parse_args(args)
    service_type, service_name, request = prepare_call(arguments)
    rclpy.init(args=None)
    node = Node("sonho_service_client_py")
    client = node.create_client(service_type, service_name)
    try:
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for /{service_name}...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.exception() is not None:
            raise future.exception()
        print(future.result())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
