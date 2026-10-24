import rclpy 
from rclpy.node import Node
from std_msgs.msg import String

class My_publisher(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.pub = self.create_publisher(String, 'talking', 10)
        self.time = self.create_timer(1.0, self.publisher_msg)
        self.counter=0

    def publisher_msg(self):
        msg = String()
        msg.data = f'Hello from ros2: {self.counter}'
        self.pub.publish(msg)    
        self.get_logger().info(f'published: {msg.data}')
        self.counter += 1   
             
def main(args=None):
    rclpy.init(args = args)
    node = My_publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()    
