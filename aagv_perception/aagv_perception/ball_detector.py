import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('hsv_lower', [5, 120, 80])
        self.declare_parameter('hsv_upper', [20, 255, 255])
        self.declare_parameter('min_radius_px', 5.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        self.min_radius = float(self.get_parameter('min_radius_px').value)

        self.bridge = CvBridge()
        self.depth = None
        self.depth_frame = None
        self.K = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/depth_image', self.depth_cb, 10)
        self.create_subscription(Image, '/camera/image', self.color_cb, 10)

        self.pub_point = self.create_publisher(PointStamped, '/ball/point', 10)
        self.pub_marker = self.create_publisher(Marker, '/ball/marker', 10)
        self.pub_mask = self.create_publisher(Image, '/ball/mask', 10)
        self.get_logger().info('ball_detector started')

    def info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_frame = msg.header.frame_id

    def color_cb(self, msg):
        if self.depth is None or self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return
        c = max(cnts, key=cv2.contourArea)
        (u, v), radius = cv2.minEnclosingCircle(c)
        if radius < self.min_radius:
            return
        u, v = int(round(u)), int(round(v))

        h, w = self.depth.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return
        z = float(self.depth[v, u])
        if not np.isfinite(z) or z <= 0.0:
            return

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        ps = PointStamped()
        ps.header.frame_id = self.depth_frame or msg.header.frame_id
        ps.header.stamp = Time().to_msg()  # latest available transform
        ps.point.x, ps.point.y, ps.point.z = x, y, z

        try:
            out = self.tf_buffer.transform(
                ps, self.target_frame, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}',
                                   throttle_duration_sec=2.0)
            return

        self.pub_point.publish(out)
        self._marker(out)
        self.get_logger().info(
            f'ball @ {self.target_frame}: '
            f'x={out.point.x:.3f} y={out.point.y:.3f} z={out.point.z:.3f}',
            throttle_duration_sec=1.0)

    def _marker(self, ps):
        m = Marker()
        m.header = ps.header
        m.ns, m.id = 'ball', 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position = ps.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 1.0
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
