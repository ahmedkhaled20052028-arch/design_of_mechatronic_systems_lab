import rclpy
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
from rclpy.node import Node
import math
import numpy as np
import sys
class Turtle_gototarget(Node):
    def __init__(self):
        super().__init__('Go_to_target_node')
        self.pose = None
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_target)
        
    def pose_callback(self, data):
        self.pose = data

    def go_to_target(self):    
        if self.pose is None:
            return
        target = Pose()
        target.x = float(sys.argv[1])
        target.y = float(sys.argv[2])
        target.theta = float(sys.argv[3])

        max_vel = Twist()
        max_vel.linear = 4
        max_vel.angular = 5

        dis_tol = 0.1
        angular_tol = 0.01
        new_vel = Twist()
        target_angle = math.atan2(target.y - self.pose.y, target.x - self.pose.x)
        dist = math.sqrt((target.x - self.pose.x)**2 + (target.y - self.pose.y)**2)
        log_msg = f'DTG: {dist} ATG: {target_angle}'
        self.get_logger().info(log_msg)

        if abs(target_angle - self.pose.theta) > angular_tol:
            new_vel.angular.z = (target_angle - self.pose.theta)
        else:
            if dist >= dis_tol:
                new_vel.linear.x = dist    
            else:
                new_vel.linear.x = 0.0
                self.get_logger().info('target reached')
                quit()
        self.pub.publish(new_vel)             


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Turtle_gototarget()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()