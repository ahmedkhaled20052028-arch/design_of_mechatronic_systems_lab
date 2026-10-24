#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty, Trigger
from turtlesim.srv import SetPen, TeleportAbsolute


class ResetServer(Node):

    def __init__(self):
        super().__init__("reset_server")

        self.cb_group = ReentrantCallbackGroup()

        self.clear_client = self.create_client(
            Empty,
            "/clear",
            callback_group=self.cb_group
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            "/turtle1/teleport_absolute",
            callback_group=self.cb_group
        )

        self.pen_client = self.create_client(
            SetPen,
            "/turtle1/set_pen",
            callback_group=self.cb_group
        )

        self.reset_service = self.create_service(
            Trigger,
            "/reset_turtle",
            self.reset_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            "Reset service ready: /reset_turtle"
        )

    def call_service(self, client, request, timeout=3.0):

        if not client.wait_for_service(timeout_sec=timeout):
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + timeout

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)

        if not future.done():
            return False

        try:
            future.result()
            return True
        except Exception as error:
            self.get_logger().error(str(error))
            return False

    def reset_callback(self, request, response):

        # Disable the pen before teleporting.
        pen_off = SetPen.Request()
        pen_off.r = 0
        pen_off.g = 0
        pen_off.b = 0
        pen_off.width = 1
        pen_off.off = 1

        # Restore the pen after teleporting.
        pen_on = SetPen.Request()
        pen_on.r = 255
        pen_on.g = 255
        pen_on.b = 255
        pen_on.width = 2
        pen_on.off = 0

        if not self.call_service(self.pen_client, pen_off):
            response.success = False
            response.message = "Failed to disable turtle pen."
            return response

        # Clear the drawing.
        if not self.call_service(
            self.clear_client,
            Empty.Request()
        ):
            self.call_service(self.pen_client, pen_on)
            response.success = False
            response.message = "Failed to clear the screen."
            return response

        # Teleport to the original position.
        teleport = TeleportAbsolute.Request()
        teleport.x = 5.544445
        teleport.y = 5.544445
        teleport.theta = 0.0

        if not self.call_service(self.teleport_client, teleport):
            self.call_service(self.pen_client, pen_on)
            response.success = False
            response.message = "Failed to teleport turtle."
            return response

        if not self.call_service(self.pen_client, pen_on):
            response.success = False
            response.message = "Reset completed, but pen restoration failed."
            return response

        response.success = True
        response.message = (
            "Screen cleared and turtle returned to its starting pose."
        )

        self.get_logger().info(response.message)

        return response


def main(args=None):

    rclpy.init(args=args)

    node = ResetServer()

    executor = MultiThreadedExecutor(num_threads=4)
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
    