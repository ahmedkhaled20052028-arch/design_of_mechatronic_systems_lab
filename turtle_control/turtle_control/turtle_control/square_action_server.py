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
from turtlesim.srv import SetPen, TeleportAbsolute

from turtle_interface.action import DrawSquare, GoToPose
from turtle_interface.msg import TargetPose


class SquareActionServer(Node):

    def __init__(self):
        super().__init__("square_action_server")

        self.cb_group = ReentrantCallbackGroup()

        # Service that clears the screen and resets the turtle.
        self.reset_client = self.create_client(
            Trigger,
            "/reset_turtle",
            callback_group=self.cb_group
        )

        # Service that controls whether the turtle draws.
        self.pen_client = self.create_client(
            SetPen,
            "/turtle1/set_pen",
            callback_group=self.cb_group
        )

        # Service that moves the turtle without drawing a path.
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            "/turtle1/teleport_absolute",
            callback_group=self.cb_group
        )

        # Action client used to move between square corners.
        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            "/go_to_pose",
            callback_group=self.cb_group
        )

        # Action server that accepts square-drawing requests.
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

        event = threading.Event()
        future.add_done_callback(lambda _: event.set())

        if not event.wait(timeout):
            self.get_logger().error(
                f"ROS operation timed out after {timeout} seconds."
            )
            return None

        try:
            return future.result()

        except Exception as exc:
            self.get_logger().error(
                f"ROS future failed: {exc}"
            )
            return None

    def wait_for_dependency(self, check_ready, timeout=10.0):

        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:

            if check_ready():
                return True

            time.sleep(0.1)

        return False

    def call_service(self, client, request, timeout=5.0):

        if not self.wait_for_dependency(
            client.service_is_ready,
            timeout=10.0
        ):
            self.get_logger().error(
                f"Service {client.srv_name} is unavailable."
            )
            return None

        future = client.call_async(request)

        return self.wait_for_future(future, timeout)

    def set_pen(self, off):

        request = SetPen.Request()

        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 2
        request.off = off

        response = self.call_service(
            self.pen_client,
            request
        )

        return response is not None

    def move_to_corner(self, x, y, yaw):

        request = TeleportAbsolute.Request()

        request.x = float(x)
        request.y = float(y)
        request.theta = float(yaw)

        response = self.call_service(
            self.teleport_client,
            request
        )

        return response is not None

    def send_pose_goal(self, x, y, yaw):

        target = TargetPose()
        target.x = float(x)
        target.y = float(y)
        target.yaw = float(yaw)

        goal = GoToPose.Goal()
        goal.target = target

        # Send the goal to the GoToPose action server.
        send_future = self.go_to_pose_client.send_goal_async(goal)

        pose_goal_handle = self.wait_for_future(
            send_future,
            timeout=10.0
        )

        if pose_goal_handle is None:
            self.get_logger().error(
                "Timed out waiting for the GoToPose goal response."
            )
            return False

        if not pose_goal_handle.accepted:
            self.get_logger().error(
                "GoToPose goal was rejected."
            )
            return False

        # Wait until the turtle reaches the requested pose.
        result_future = pose_goal_handle.get_result_async()

        pose_result = self.wait_for_future(
            result_future,
            timeout=90.0
        )

        if pose_result is None:
            self.get_logger().error(
                "Timed out waiting for the GoToPose result."
            )
            return False

        if pose_result.status != 4:
            self.get_logger().error(
                f"GoToPose finished with status "
                f"{pose_result.status}."
            )
            return False

        if not pose_result.result.success:
            self.get_logger().error(
                "GoToPose reported failure."
            )
            return False

        return True

    def execute_callback(self, goal_handle):

        result = DrawSquare.Result()
        side = float(goal_handle.request.side_length)

        try:

            # 1. Wait for all required services and the action server.
            if not self.wait_for_dependency(
                self.reset_client.service_is_ready,
                timeout=10.0
            ):
                result.success = False
                result.message = "Reset service is unavailable."
                goal_handle.abort()
                return result

            if not self.wait_for_dependency(
                self.pen_client.service_is_ready,
                timeout=10.0
            ):
                result.success = False
                result.message = "SetPen service is unavailable."
                goal_handle.abort()
                return result

            if not self.wait_for_dependency(
                self.teleport_client.service_is_ready,
                timeout=10.0
            ):
                result.success = False
                result.message = "Teleport service is unavailable."
                goal_handle.abort()
                return result

            if not self.wait_for_dependency(
                self.go_to_pose_client.server_is_ready,
                timeout=10.0
            ):
                result.success = False
                result.message = "GoToPose action server is unavailable."
                goal_handle.abort()
                return result

            # 2. Reset the turtle and clear the drawing.
            reset_response = self.call_service(
                self.reset_client,
                Trigger.Request()
            )

            if reset_response is None or not reset_response.success:
                result.success = False
                result.message = "Failed to reset the turtle."
                goal_handle.abort()
                return result

            # 3. Lift the pen so positioning does not draw a diagonal.
            if not self.set_pen(off=True):
                result.success = False
                result.message = "Failed to lift the pen."
                goal_handle.abort()
                return result

            # 4. Calculate the four corners around the center.
            center_x = 5.544445
            center_y = 5.544445
            half = side / 2.0

            bottom_left = (
                center_x - half,
                center_y - half
            )

            bottom_right = (
                center_x + half,
                center_y - half
            )

            top_right = (
                center_x + half,
                center_y + half
            )

            top_left = (
                center_x - half,
                center_y + half
            )

            # 5. Position the turtle at the first corner WITHOUT drawing.
            start_x, start_y = bottom_left

            if not self.move_to_corner(
                start_x,
                start_y,
                0.0
            ):
                result.success = False
                result.message = "Failed to position the turtle."
                goal_handle.abort()
                return result

            # 6. Lower the pen to begin drawing.
            if not self.set_pen(off=False):
                result.success = False
                result.message = "Failed to lower the pen."
                goal_handle.abort()
                return result

            # 7. Visit the remaining corners and return to the start.
            #
            # Each GoToPose goal draws one side of the square.
            #
            # Start: bottom-left
            # Side 1: bottom-left  -> bottom-right
            # Side 2: bottom-right -> top-right
            # Side 3: top-right    -> top-left
            # Side 4: top-left     -> bottom-left

            corners = [
                (bottom_right[0], bottom_right[1], 0.0),
                (top_right[0], top_right[1], math.pi / 2.0),
                (top_left[0], top_left[1], math.pi),
                (bottom_left[0], bottom_left[1], -math.pi / 2.0),
            ]

            for index, (x, y, yaw) in enumerate(
                corners,
                start=1
            ):

                # Handle cancellation before starting another side.
                if goal_handle.is_cancel_requested:

                    self.set_pen(off=True)

                    result.success = False
                    result.message = "Square drawing canceled."

                    goal_handle.canceled()
                    return result

                # Publish feedback.
                feedback = DrawSquare.Feedback()
                feedback.side_number = index
                feedback.message = (
                    f"Drawing side {index}/4."
                )

                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"Drawing side {index}/4: "
                    f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
                )

                # Execute exactly one GoToPose goal for this side.
                if not self.send_pose_goal(x, y, yaw):

                    self.set_pen(off=True)

                    result.success = False
                    result.message = (
                        f"Failed to draw side {index}/4."
                    )

                    goal_handle.abort()
                    return result

            # 8. The turtle has returned to the starting corner.
            goal_handle.succeed()

            result.success = True
            result.message = (
                "Square completed using exactly four GoToPose goals."
            )

            self.get_logger().info(
                "Square completed successfully."
            )

            return result

        except Exception as exc:

            self.get_logger().error(
                f"DrawSquare execution failed: {exc}"
            )

            self.set_pen(off=True)

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
