import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from apriltag import apriltag

fx, fy, cx, cy = 7.7995083991378465e2, 7.7995083991378465e2, 3.2135820800218698e2, 2.3265993020995523e2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('tag_detect')
        self.subscription = self.create_subscription(Image, '/mono_left_unsync', self.callback, QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.dir_pub = self.create_publisher(Vector3, "/tag_dir_frd", QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.detector = apriltag("tag36h11")
        self.cnt = 0
    def callback(self, msg):
        self.cnt += 1
        if self.cnt > 4:
            self.cnt = 0
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            detections = self.detector.detect(image)
            if detections:
                center = detections[0]['center']
                x = (center[0] - cx) / fx
                y = (center[1] - cy) / fy
                z = 1.0
                v = np.array([x, y, z])
                dir = v / np.linalg.norm(v)
                print(dir)
                dir_msg = Vector3()
                dir_msg.x = dir[2]
                dir_msg.y = dir[0]
                dir_msg.z = dir[1]
                self.dir_pub.publish(dir_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
