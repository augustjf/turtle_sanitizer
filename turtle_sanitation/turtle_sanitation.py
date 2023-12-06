import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

class TurtleSanitizer(Node):
    def __init__(self):
        super().__init__(node_name='turtle_sanitizer')
        self.map = None
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

    
    def map_callback(self, msg):
        self.map = msg

def main():
    rclpy.init()
    turtle_sanitizer = TurtleSanitizer()
    rclpy.spin(turtle_sanitizer)

if __name__ == '__main__':
    main()