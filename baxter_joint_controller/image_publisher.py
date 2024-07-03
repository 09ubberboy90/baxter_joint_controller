import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class MinimalPublisher(Node):
      def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, '/robot/xdisplay', 10)
        self.cv_image = cv2.imread('/home/ubb/Pictures/cvas.png') ### an RGB image 
        timer_period =1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

      def timer_callback(self):
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
        self.get_logger().info('Publishing an image')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
