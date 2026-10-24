import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

class My_subscriper(Node):
    def __init__(self):
        super().__init__('subscriper_node')
        self.sub = self.create_subscription(String, 'talking', self.talking_callback, 10)

    def talking_callback(self,msg):
        self.get_logger().info(f'recieved {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = My_subscriper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
