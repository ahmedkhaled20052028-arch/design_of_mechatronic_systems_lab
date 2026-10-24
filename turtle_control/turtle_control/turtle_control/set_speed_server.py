#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

from turtle_interface.srv import SetSpeedLimits


class SetSpeedServer(Node):

    def __init__(self):
        super().__init__("set_speed_server")

        self.cb_group = ReentrantCallbackGroup()

        self.parameter_client = self.create_client(
            SetParameters,
            "/go_to_pose_server/set_parameters",
            callback_group=self.cb_group
        )

        self.speed_service = self.create_service(
            SetSpeedLimits,
            "/set_speed_limits",
            self.speed_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            "Speed service ready: /set_speed_limits"
        )

    def speed_callback(self, request, response):

        linear = float(request.max_linear_speed)
        angular = float(request.max_angular_speed)

        # Validate the requested speeds.
        if not (
            0.0 < linear <= 10.0
            and 0.0 < angular <= 10.0
        ):
            response.success = False
            response.message = (
                "Both speeds must be greater than zero "
                "and no greater than 10."
            )
            return response

        # Ensure the target node is available.
        if not self.parameter_client.wait_for_service(
            timeout_sec=2.0
        ):
            response.success = False
            response.message = (
                "Cannot reach the go_to_pose parameter service."
            )
            return response

        request_parameters = SetParameters.Request()

        # Prepare the linear speed parameter.
        linear_parameter = Parameter()
        linear_parameter.name = "max_linear_speed"
        linear_parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=linear
        )

        # Prepare the angular speed parameter.
        angular_parameter = Parameter()
        angular_parameter.name = "max_angular_speed"
        angular_parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=angular
        )

        request_parameters.parameters = [
            linear_parameter,
            angular_parameter
        ]

        future = self.parameter_client.call_async(
            request_parameters
        )

        deadline = time.monotonic() + 3.0

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)

        if not future.done():
            response.success = False
            response.message = "Parameter update timed out."
            return response

        try:
            results = future.result().results

            response.success = all(
                result.successful for result in results
            )

            if response.success:
                response.message = (
                    "Linear and angular speed limits updated."
                )
            else:
                response.message = (
                    "ROS 2 rejected one or more parameter changes."
                )

        except Exception as error:
            response.success = False
            response.message = f"Update failed: {error}"

        return response


def main(args=None):

    rclpy.init(args=args)

    node = SetSpeedServer()

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