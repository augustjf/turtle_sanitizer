import rclpy
import ros2_numpy as rnp
import numpy as np
import cv2 

from tf2_ros.transform_listener import TransformListener
#from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
#from geographic_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TurtleSanitizer(Node):
    def __init__(self):
        super().__init__(node_name='turtle_sanitizer')
        self.map = OccupancyGrid
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.map_publisher = self.create_publisher(Image, 'map_image', 10)
        self.map_timer = self.create_timer(1, self.map_timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.np_ma_map = np.ma.array(0) #Masked array of map
        self.npmap = np.array(0)
        self.bridge = CvBridge()
        self.map_recieved = False
    
    def thresh_and_fix_map(self):
        data = np.asarray(self.map.data, dtype=np.int16).reshape(self.map.info.height, self.map.info.width)
        data = 255-data*255/100 #Normalize image to 0-255
        data[data > 255] = 127 #Value of unknown pixels in the occupancy grid
        data = data.astype(np.uint8)
        data[data < 120] = 0 #Lower threshold
        data[data > 160] = 255 #Higher threshold
        self.npmap = (np.array(data))

    def map_callback(self, msg):
        self.map = msg
        self.np_ma_map = rnp.numpify(msg)
        self.map_recieved = True
    
    def map_timer_callback(self):
        self.thresh_and_fix_map()
        if self.map_recieved:
            self.map_publisher.publish(self.bridge.cv2_to_imgmsg(np.array(self.npmap)))
    
    def timer_callback(self):
        from_frame = 'map'
        to_frame = 'base_link'
        # try:
        #     t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')
    
def main():
    rclpy.init()
    turtle_sanitizer = TurtleSanitizer()
    rclpy.spin(turtle_sanitizer)

if __name__ == '__main__':
    main()