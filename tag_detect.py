import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from apriltag import apriltag

class ImageSubscriber(Node):
    def __init__(self, fx, fy, cx, cy):
        super().__init__('tag_detect')
        self.subscription = self.create_subscription(Image, '/mono_left', self.callback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.dir_pub = self.create_publisher(Vector3, "/tag_dir_frd", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.detector = apriltag("tag36h11")
        self.cnt = 0
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
    def callback(self, msg):
        self.cnt += 1
        if self.cnt > 5:
            self.cnt = 0
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            tags = self.detector.detect(image)
            if tags:
                for tag in tags:
                    if tag['id'] == 0:
                        center = tag['center']
                        x = (center[0] - self.cx) / self.fx
                        y = (center[1] - self.cy) / self.fy
                        z = 1.0
                        v = np.array([x, y, z])
                        dir = v / np.linalg.norm(v)
                        print(dir)
                        dir_msg = Vector3()
                        dir_msg.x = dir[2]
                        dir_msg.y = dir[0]
                        dir_msg.z = dir[1]
                    elif tag['id'] == 1:
                        print("tag 1 found")
                        dir_msg = Vector3()
                        break
                self.dir_pub.publish(dir_msg)

def main(args=None):
    rclpy.init(args=args)
    with open(args[1], 'r') as f:
        fx, fy, cx, cy = map(float, f.read().strip().split(','))
    node = ImageSubscriber(fx, fy, cx, cy)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
