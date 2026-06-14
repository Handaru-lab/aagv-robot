#!/usr/bin/env python3
"""
dark_object_detector.py — Deteksi objek gelap (penghapus hitam) di atas meja terang.
Strategi: grayscale threshold (Otsu) → morphological close → kontur terbesar →
centroid → depth deproject → publish PointStamped di camera_color_optical_frame.

Topik subscribe:
  /camera/camera/color/image_raw        (sensor_msgs/Image, BGR8/RGB8)
  /camera/camera/depth/image_rect_raw   (sensor_msgs/Image, 16UC1, mm)
  /camera/camera/color/camera_info      (sensor_msgs/CameraInfo)

Topik publish:
  /object/point     (geometry_msgs/PointStamped)  — posisi 3D objek di frame kamera
  /object/marker    (visualization_msgs/Marker)   — sphere marker untuk RViz
  /object/mask      (sensor_msgs/Image)           — mask debug (mono8)
  /object/annotated (sensor_msgs/Image)           — frame dengan overlay deteksi

Parameter (bisa di-override via launch/CLI):
  gray_threshold:     int, 0=Otsu otomatis (default), >0=manual threshold
  min_area:           int, minimum contour area in pixels (default 1500)
  roi_top_fraction:   float, ignore top fraction of frame (default 0.30)
  depth_margin:       int, depth sampling radius around centroid (default 5)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import message_filters


class DarkObjectDetector(Node):
    def __init__(self):
        super().__init__('dark_object_detector')

        # Parameters
        self.declare_parameter('gray_threshold', 0)       # 0 = Otsu
        self.declare_parameter('min_area', 1500)
        self.declare_parameter('roi_top_fraction', 0.30)
        self.declare_parameter('depth_margin', 5)

        self.br = CvBridge()
        self.intrinsics = None  # (fx, fy, cx, cy)

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.cb_info, 10)

        sub_color = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data)
        sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw',
            qos_profile=qos_profile_sensor_data)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.cb_frames)

        # Publishers
        self.pub_point = self.create_publisher(PointStamped, '/object/point', 10)
        self.pub_marker = self.create_publisher(Marker, '/object/marker', 10)
        self.pub_mask = self.create_publisher(Image, '/object/mask', 1)
        self.pub_annotated = self.create_publisher(Image, '/object/annotated', 1)

        self.get_logger().info('DarkObjectDetector started. Waiting for frames...')

    def cb_info(self, msg):
        if self.intrinsics is None:
            K = msg.k
            self.intrinsics = (K[0], K[4], K[2], K[5])  # fx, fy, cx, cy
            self.get_logger().info(
                f'Camera intrinsics: fx={K[0]:.1f} fy={K[4]:.1f} cx={K[2]:.1f} cy={K[5]:.1f}')

    def cb_frames(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        # Get params
        thr_param = self.get_parameter('gray_threshold').value
        min_area = self.get_parameter('min_area').value
        roi_top = self.get_parameter('roi_top_fraction').value
        d_margin = self.get_parameter('depth_margin').value
        fx, fy, cx, cy = self.intrinsics

        # Convert images
        color = self.br.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.br.imgmsg_to_cv2(depth_msg, 'passthrough')  # 16UC1, mm
        H, W = color.shape[:2]

        # Grayscale
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # ROI: mask out top fraction (background clutter)
        roi_y = int(H * roi_top)
        gray_roi = gray.copy()
        gray_roi[:roi_y, :] = 255  # force top region to white (not dark)

        # Threshold
        if thr_param > 0:
            _, mask = cv2.threshold(gray_roi, thr_param, 255, cv2.THRESH_BINARY_INV)
        else:
            # Otsu on the ROI region only for adaptive threshold
            roi_pixels = gray_roi[roi_y:, :]
            otsu_val, _ = cv2.threshold(roi_pixels, 0, 255,
                                         cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            _, mask = cv2.threshold(gray_roi, otsu_val, 255, cv2.THRESH_BINARY_INV)
            # Clamp: don't let Otsu go above 120 (would pick up table)
            if otsu_val > 120:
                _, mask = cv2.threshold(gray_roi, 95, 255, cv2.THRESH_BINARY_INV)

        # Morphological close to bridge the white label gap
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # Open to remove small noise
        kernel_s = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_s)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if cv2.contourArea(c) >= min_area]

        # Publish mask
        self.pub_mask.publish(self.br.cv2_to_imgmsg(mask, 'mono8'))

        if not big:
            return

        # Largest contour = target
        target = max(big, key=cv2.contourArea)
        M = cv2.moments(target)
        if M['m00'] == 0:
            return
        px = int(M['m10'] / M['m00'])
        py = int(M['m01'] / M['m00'])

        # Depth: black objects absorb IR → depth=0 on surface.
        # Strategy: sample depth in a RING around the contour (= table surface),
        # then subtract object height to get object top position.
        OBJECT_HEIGHT_M = 0.04  # penghapus ~4cm tall

        # First try centroid directly
        y_lo = max(0, py - d_margin)
        y_hi = min(H, py + d_margin + 1)
        x_lo = max(0, px - d_margin)
        x_hi = min(W, px + d_margin + 1)
        depth_patch = depth[y_lo:y_hi, x_lo:x_hi]
        valid_center = depth_patch[depth_patch > 0]

        if len(valid_center) > 0:
            z_mm = float(np.median(valid_center))
            z_m = z_mm / 1000.0
        else:
            # Fallback: sample ring around contour (table surface)
            obj_mask = np.zeros((H, W), dtype=np.uint8)
            cv2.drawContours(obj_mask, [target], -1, 255, -1)
            # Dilate to get ring on table
            dilate_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (40, 40))
            dilated = cv2.dilate(obj_mask, dilate_k)
            ring_mask = dilated & (~obj_mask)
            ring_depth = depth[ring_mask > 0]
            valid_ring = ring_depth[ring_depth > 0]
            if len(valid_ring) == 0:
                self.get_logger().warn(
                    'No valid depth at centroid or ring', throttle_duration_sec=2.0)
                return
            table_z_mm = float(np.median(valid_ring))
            # Object top is closer to camera than table by object height
            z_mm = table_z_mm - (OBJECT_HEIGHT_M * 1000.0)
            z_m = z_mm / 1000.0
            self.get_logger().info(
                f'Depth from ring: table={table_z_mm:.0f}mm, obj_top={z_mm:.0f}mm',
                throttle_duration_sec=5.0)

        # Deproject pixel to 3D (pinhole model)
        x_m = (px - cx) * z_m / fx
        y_m = (py - cy) * z_m / fy

        # Publish PointStamped
        pt = PointStamped()
        pt.header = color_msg.header
        pt.point.x = x_m
        pt.point.y = y_m
        pt.point.z = z_m
        self.pub_point.publish(pt)

        # Publish Marker (sphere)
        mk = Marker()
        mk.header = color_msg.header
        mk.ns = 'dark_object'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = x_m
        mk.pose.position.y = y_m
        mk.pose.position.z = z_m
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.05
        mk.color.r = 0.2
        mk.color.g = 0.2
        mk.color.b = 0.2
        mk.color.a = 0.9
        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 200_000_000
        self.pub_marker.publish(mk)

        # Publish annotated image
        vis = color.copy()
        cv2.drawContours(vis, [target], -1, (0, 255, 0), 2)
        cv2.circle(vis, (px, py), 6, (0, 0, 255), -1)
        label = f'({x_m:.3f}, {y_m:.3f}, {z_m:.3f})m'
        cv2.putText(vis, label, (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        area = cv2.contourArea(target)
        cv2.putText(vis, f'A={area:.0f}px', (px + 10, py + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        self.pub_annotated.publish(self.br.cv2_to_imgmsg(vis, 'bgr8'))


def main():
    rclpy.init()
    node = DarkObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
