import rclpy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import pyplot



class TurtleSanitizer(Node):
    def __init__(self):
        super().__init__(node_name='turtle_sanitizer')
        self.map = OccupancyGrid
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.image_publisher = self.create_publisher(Image, 'map_image', 10)
        self.map_timer = self.create_timer(1, self.map_timer_callback)
        self.map_data = np.array(0)
        self.npmap = np.array(0)
        self.bridge = CvBridge()
        self.map_recieved = False
    
    def translate_map_data(self):
        data = np.asarray(self.map.data, dtype=np.int16)
        data = 255-data*255/100 #Normalize image to 0-255     
        data[data > 255] = -1 #Value of unknown pixels in the occupancy grid        
        self.map_data = data

    def save_map_histogram(self):
        n_bins = 257
        pyplot.hist(self.map_data, bins=n_bins)
        pyplot.savefig('hist.png')
    
    def make_map_image(self):
        data = self.map_data.reshape(self.map.info.height, self.map.info.width)
        self.npmap = data
    
    def thresh_map_data(self):
        data = self.map_data
        data[data < 0] = 127
        data[data < 120] = 0
        data[data > 140] = 255
        data = np.array(data, dtype=np.uint8)
        self.map_data = data

    def map_callback(self, msg):
        self.map = msg
        self.map_recieved = True
        self.get_logger().info('Map recieved')
    
    def map_timer_callback(self):
        if self.map_recieved:
            self.translate_map_data()
            self.thresh_map_data()
            self.make_map_image()
            #self.save_map_histogram()
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(np.array(self.npmap)))
            self.get_logger().info('Publishing')
    
def main():
    rclpy.init()
    turtle_sanitizer = TurtleSanitizer()
    rclpy.spin(turtle_sanitizer)

if __name__ == '__main__':
    main()