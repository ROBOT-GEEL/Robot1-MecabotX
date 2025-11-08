import rclpy
from rclpy.node import Node

class Nogo(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.get_logger().info('Hallo van demo_node!')

def main(args=None):
    rclpy.init(args=args)
    node = Nogo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

