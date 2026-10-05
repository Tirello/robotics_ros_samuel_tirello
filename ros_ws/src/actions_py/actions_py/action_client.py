"""Command-line client for CountUp and cancelable Timer actions."""

from __future__ import annotations

import argparse
from typing import Sequence

import rclpy
from custom_interfaces.action import CountUp
from custom_interfaces.action import Timer
from rclpy.action import ActionClient
from rclpy.node import Node


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    count = commands.add_parser("count")
    count.add_argument("limit", type=int)
    timer = commands.add_parser("timer")
    timer.add_argument("duration", type=float)
    timer.add_argument("--cancel-after", type=float, default=None)
    return parser


def main(args: Sequence[str] | None = None) -> None:
    arguments = build_parser().parse_args(args)
    rclpy.init(args=None)
    node = Node("sonho_action_client_py")
    cancel_timer = None
    try:
        if arguments.operation == "count":
            action_type = CountUp
            action_name = "count_up"
            goal = CountUp.Goal(limit=arguments.limit)

            def feedback(message):
                print(f"Feedback: {message.feedback.current_value}")

        else:
            action_type = Timer
            action_name = "timer"
            goal = Timer.Goal(duration=arguments.duration)

            def feedback(message):
                print(f"Feedback: {message.feedback.elapsed:.3f} s")

        client = ActionClient(node, action_type, action_name)
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action server /{action_name} is not available")
        send_future = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Goal rejected.")
            return

        if arguments.operation == "timer" and arguments.cancel_after is not None:
            canceled = False

            def request_cancel():
                nonlocal canceled
                if not canceled:
                    canceled = True
                    print("Cancellation requested.")
                    goal_handle.cancel_goal_async()
                    cancel_timer.cancel()

            cancel_timer = node.create_timer(max(arguments.cancel_after, 0.001), request_cancel)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        print(wrapped.result)
    finally:
        if cancel_timer is not None:
            node.destroy_timer(cancel_timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
