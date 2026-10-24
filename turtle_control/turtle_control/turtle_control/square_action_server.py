#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from turtle_interface.action import DrawSquare, GoToPose
from turtle_interface.msg import TargetPose


class SquareActionServer(Node):

    def __init__(self):
        super().__init__("square_action_server")

        self.cb_group = ReentrantCallbackGroup()

        self.reset_client = self.create_client(
            Trigger,
            "/reset_turtle",
            callback_group=self.cb_group
        )

        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            "/go_to_pose",
            callback_group=self.cb_group
        )

        self.square_server = ActionServer(
            self,
            DrawSquare,
            "/draw_square",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            "Square action server ready: /draw_square"
        )

    def goal_callback(self, request):
        if not 0.2 <= request.side_length <= 4.0:
            self.get_logger().warning(
                "Side length must be between 0.2 and 4.0."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def wait_for_future(self, future, timeout):
        """Wait while other executor threads process ROS callbacks."""
        event = threading.Event()

        future.add_done_callback(lambda _: event.set())

        if not event.wait(timeout):
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(
                f"ROS future failed: {exc}"
            )
            return None

    def wait_for_dependency(self, check_ready, timeout=5.0):
        """Poll readiness without blocking the ROS executor."""
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if check_ready():
                return True

            time.sleep(0.1)

        return False

    def execute_callback(self, goal_handle):
        result = DrawSquare.Result()
        side = float(goal_handle.request.side_length)

        try:
            # 1. Wait for the reset service.
            if not self.wait_for_dependency(
                self.reset_client.service_is_ready,
                timeout=10.0
                ):
                self.get_logger().error(
                    "Reset service not ready. Check /reset_turtle and ROS_DOMAIN_ID."
                )
                result.success = False
                result.message = "Reset service is unavailable."
                goal_handle.abort()
                return result
                result.success = False
                result.message = "Reset service is unavailable."
                goal_handle.abort()
                return result

            # 2. Reset the turtle.
            reset_future = self.reset_client.call_async(
                Trigger.Request()
            )

            reset_response = self.wait_for_future(
                reset_future,
                timeout=5.0
            )

            if reset_response is None or not reset_response.success:
                result.success = False
                result.message = "Reset failed."
                goal_handle.abort()
                return result

            # 3. Wait for GoToPose.
            if not self.wait_for_dependency(
                self.go_to_pose_client.server_is_ready,
                timeout=5.0
            ):
                result.success = False
                result.message = "GoToPose action server is unavailable."
                goal_handle.abort()
                return result

            # 4. Calculate four square corners.
            center_x = 5.544445
            center_y = 5.544445
            half = side / 2.0

            corners = [
                (center_x + half, center_y - half),
                (center_x + half, center_y + half),
                (center_x - half, center_y + half),
                (center_x - half, center_y - half),
            ]

            headings = [
                math.pi / 2.0,
                math.pi,
                -math.pi / 2.0,
                0.0,
            ]

            # 5. Visit each corner in order.
            for index, ((x, y), yaw) in enumerate(
                zip(corners, headings), start=1
            ):
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = "Square drawing canceled."
                    goal_handle.canceled()
                    return result

                feedback = DrawSquare.Feedback()
                feedback.side_number = index
                feedback.message = f"Moving to corner {index}/4."
                goal_handle.publish_feedback(feedback)

                target = TargetPose()
                target.x = x
                target.y = y
                target.yaw = yaw

                goal = GoToPose.Goal()
                goal.target = target

                send_future = self.go_to_pose_client.send_goal_async(
                    goal
                )

                pose_goal_handle = self.wait_for_future(
                    send_future,
                    timeout=10.0
                )

                if pose_goal_handle is None or not pose_goal_handle.accepted:
                    result.success = False
                    result.message = (
                        f"GoToPose goal {index} was rejected or timed out."
                    )
                    goal_handle.abort()
                    return result

                pose_result_future = pose_goal_handle.get_result_async()

                pose_result = self.wait_for_future(
                    pose_result_future,
                    timeout=90.0
                )

                if (
                    pose_result is None
                    or not pose_result.result.success
                ):
                    result.success = False
                    result.message = (
                        f"GoToPose goal {index} failed or timed out."
                    )
                    goal_handle.abort()
                    return result

            goal_handle.succeed()
            result.success = True
            result.message = (
                "Square completed using exactly four GoToPose goals."
            )
            return result

        except Exception as exc:
            self.get_logger().error(
                f"DrawSquare execution failed: {exc}"
            )

            result.success = False
            result.message = f"Execution error: {exc}"
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)

    node = SquareActionServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()