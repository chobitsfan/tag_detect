import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from apriltag import apriltag

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('tag_detect')
        self.subscription = self.create_subscription(Image, '/mono_left_unsync', self.callback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.detector = apriltag("tag36h11")
        self.cnt = 0
    def callback(self, msg):
        self.cnt += 1
        if self.cnt > 4:
            self.cnt = 0
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            detections = self.detector.detect(image)
            if detections:
                print(detections)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
