"""CountUp and cancelable Timer action servers."""

from __future__ import annotations

import math
import time
from typing import Sequence

import rclpy
from custom_interfaces.action import CountUp
from custom_interfaces.action import Timer
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ActionServers(Node):
    """Host both official action exercises in one multithreaded node."""

    def __init__(self) -> None:
        super().__init__("sonho_action_servers_py")
        callback_group = ReentrantCallbackGroup()
        self._count_server = ActionServer(
            self,
            CountUp,
            "count_up",
            execute_callback=self.execute_count,
            goal_callback=self.count_goal,
            cancel_callback=self.accept_cancel,
            callback_group=callback_group,
        )
        self._timer_server = ActionServer(
            self,
            Timer,
            "timer",
            execute_callback=self.execute_timer,
            goal_callback=self.timer_goal,
            cancel_callback=self.accept_cancel,
            callback_group=callback_group,
        )
        self.get_logger().info("CountUp and Timer action servers are ready (Python).")

    @staticmethod
    def count_goal(goal_request: CountUp.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT if goal_request.limit >= 0 else GoalResponse.REJECT

    @staticmethod
    def timer_goal(goal_request: Timer.Goal) -> GoalResponse:
        valid = math.isfinite(goal_request.duration) and goal_request.duration > 0.0
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    @staticmethod
    def accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    @staticmethod
    def execute_count(goal_handle) -> CountUp.Result:
        last_value = 0
        time.sleep(0.05)
        for current_value in range(1, goal_handle.request.limit + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return CountUp.Result(
                    success=False,
                    final_value=last_value,
                    message="Count canceled.",
                )
            last_value = current_value
            goal_handle.publish_feedback(CountUp.Feedback(current_value=current_value))
            time.sleep(0.25)
        goal_handle.succeed()
        return CountUp.Result(
            success=True,
            final_value=last_value,
            message="Count completed successfully.",
        )

    @staticmethod
    def execute_timer(goal_handle) -> Timer.Result:
        requested_duration = goal_handle.request.duration
        start = time.monotonic()
        elapsed = 0.0
        next_feedback = min(1.0, requested_duration)
        while elapsed < requested_duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Timer.Result(
                    completed=False,
                    elapsed=elapsed,
                    message="Timer canceled before completion.",
                )
            time.sleep(0.05)
            elapsed = time.monotonic() - start
            if elapsed >= next_feedback or elapsed >= requested_duration:
                goal_handle.publish_feedback(
                    Timer.Feedback(elapsed=min(elapsed, requested_duration))
                )
                next_feedback += 1.0
        goal_handle.succeed()
        return Timer.Result(
            completed=True,
            elapsed=requested_duration,
            message="Timer completed.",
        )

    def destroy_node(self) -> None:
        self._count_server.destroy()
        self._timer_server.destroy()
        super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActionServers()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
