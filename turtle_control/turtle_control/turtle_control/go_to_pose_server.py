#!/usr/bin/env python3

import math
import asyncio
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_interface.action import GoToPose


def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    """Keep a value within the specified limits."""
    return max(minimum, min(maximum, value))


class GoToPoseServer(Node):

    def __init__(self):
        super().__init__('go_to_pose_server')

        # Movement parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('position_tolerance', 0.10)
        self.declare_parameter('yaw_tolerance', 0.05)

        # If the target direction differs by more than this angle,
        # rotate in place before moving forward.
        self.declare_parameter('rotate_only_angle', 0.7)

        self.current_pose = None

        self.wait_event = threading.Event()

        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to the turtle's current pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        # Publish velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Create the GoToPose action server
        self.action_server = ActionServer(
            self,
            GoToPose,
            '/go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'GoToPose action server is ready on /go_to_pose.'
        )

    def pose_callback(self, msg):
        """Store the latest turtle pose."""
        self.current_pose = msg

    def goal_callback(self, goal_request):
        """Validate the requested target pose."""

        target = goal_request.target

        if not all(
            math.isfinite(value)
            for value in (target.x, target.y, target.yaw)
        ):
            self.get_logger().warning(
                'Goal rejected: coordinates and yaw must be finite.'
            )
            return GoalResponse.REJECT

        if not (
            0.0 <= target.x <= 11.0
            and 0.0 <= target.y <= 11.0
        ):
            self.get_logger().warning(
                'Goal rejected: x and y must be between 0 and 11.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted goal: x={target.x:.2f}, '
            f'y={target.y:.2f}, yaw={target.yaw:.2f}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow the client to cancel the goal."""
        self.get_logger().info('Cancellation requested.')
        return CancelResponse.ACCEPT

    def publish_stop(self):
        """Publish zero velocity to stop the turtle."""
        self.velocity_publisher.publish(Twist())

    def execute_callback(self, goal_handle):
       
        """Move the turtle to the requested position and orientation."""
        self.get_logger().info('EXECUTE CALLBACK STARTED')
        self.get_logger().info('Executing GoToPose goal.')

        result = GoToPose.Result()
        
        feedback = GoToPose.Feedback()

        target = goal_handle.request.target

        target_x = target.x
        target_y = target.y
        target_yaw = normalize_angle(target.yaw)

        # Wait for the first pose message
        while rclpy.ok() and self.current_pose is None:

            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()

                result.success = False
                result.message = 'Goal canceled while waiting for pose.'
                return result

            self.wait_event.wait(timeout=0.05)

        # Main control loop
        while rclpy.ok():

            # Handle cancellation
            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()

                result.success = False
                result.message = 'Goal canceled by the client.'

                self.get_logger().info(result.message)
                return result

            pose = self.current_pose

            # Calculate distance to the target
            dx = target_x - pose.x
            dy = target_y - pose.y

            distance = math.hypot(dx, dy)

            # Read speed and tolerance parameters
            max_linear_speed = float(
                self.get_parameter('max_linear_speed').value
            )

            max_angular_speed = float(
                self.get_parameter('max_angular_speed').value
            )

            position_tolerance = float(
                self.get_parameter('position_tolerance').value
            )

            yaw_tolerance = float(
                self.get_parameter('yaw_tolerance').value
            )

            rotate_only_angle = float(
                self.get_parameter('rotate_only_angle').value
            )

            # Publish feedback
            
            feedback.remaining_distance = float(distance)

            # Stage 1: Move toward the target position
            if distance > position_tolerance:

                # Desired direction from the turtle to the target
                desired_yaw = math.atan2(dy, dx)

                # Difference between desired and current heading
                angle_error = normalize_angle(
                    desired_yaw - pose.theta
                )

                feedback.angle_error = float(angle_error)
                goal_handle.publish_feedback(feedback)

                cmd = Twist()

                # Always correct the heading toward the target
                cmd.angular.z = clamp(
                    4.0 * angle_error,
                    -max_angular_speed,
                    max_angular_speed
                )

                # Rotate in place only when the heading error is large.
                # Otherwise, move forward while correcting the heading.
                if abs(angle_error) > rotate_only_angle:
                    cmd.linear.x = 0.0

                else:
                    # Reduce forward speed when the turtle is not
                    # perfectly aligned with the target.
                    alignment_factor = max(
                        0.0,
                        math.cos(angle_error)
                    )

                    desired_speed = min(
                        max_linear_speed,
                        1.5 * distance
                    )

                    cmd.linear.x = (
                        desired_speed * alignment_factor
                    )
                    
                self.velocity_publisher.publish(cmd)

            # Stage 2: Position reached; correct final orientation
            else:

                final_angle_error = normalize_angle(
                    target_yaw - pose.theta
                )

                feedback.remaining_distance = float(distance)
                feedback.angle_error = float(final_angle_error)

                goal_handle.publish_feedback(feedback)

                # Stop when both position and final orientation
                # are within their tolerances.
                if abs(final_angle_error) <= yaw_tolerance:

                    self.publish_stop()

                    goal_handle.succeed()

                    result.success = True
                    result.message = (
                        'Target position and orientation reached.'
                    )

                    self.get_logger().info(result.message)
                    return result

                # Rotate toward the requested final orientation
                cmd = Twist()

                cmd.linear.x = 0.0

                cmd.angular.z = clamp(
                    4.0 * final_angle_error,
                    -max_angular_speed,
                    max_angular_speed
                )

                self.velocity_publisher.publish(cmd)

            # Let ROS process other callbacks
            self.wait_event.wait(timeout=0.05)

        # Stop the turtle if ROS is shutting down
        self.publish_stop()

        result.success = False
        result.message = 'ROS shutdown before the goal was completed.'

        return result


def main(args=None):
    rclpy.init(args=args)

    node = GoToPoseServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()