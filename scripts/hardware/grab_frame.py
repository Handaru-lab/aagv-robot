#!/usr/bin/env python3
"""Capture satu frame color + depth dari topik RealSense, simpan sebagai PNG.
Jalankan saat rs_launch.py dan arm bringup sudah aktif.
Letakkan penghapus hitam di meja dalam jangkauan pandang kamera."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import sys, os

class FrameGrabber(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self.br = CvBridge()
        self.color_img = None
        self.depth_img = None
        self.sub_c = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.cb_color, 1)
        self.sub_d = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.cb_depth, 1)
        self.timer = self.create_timer(0.5, self.check)

    def cb_color(self, msg):
        if self.color_img is None:
            self.color_img = self.br.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(f'Color captured: {self.color_img.shape}')

    def cb_depth(self, msg):
        if self.depth_img is None:
            self.depth_img = self.br.imgmsg_to_cv2(msg, 'passthrough')
            self.get_logger().info(f'Depth captured: {self.depth_img.shape}, dtype={self.depth_img.dtype}')

    def check(self):
        if self.color_img is not None and self.depth_img is not None:
            out = os.path.expanduser('~')
            # Save color
            cv2.imwrite(f'{out}/cam_color.png', self.color_img)
            # Save depth as 16-bit PNG (raw mm) + colorized version for viewing
            cv2.imwrite(f'{out}/cam_depth_raw.png', self.depth_img)
            d_norm = cv2.normalize(self.depth_img, None, 0, 255, cv2.NORM_MINMAX)
            d_color = cv2.applyColorMap(d_norm.astype(np.uint8), cv2.COLORMAP_JET)
            cv2.imwrite(f'{out}/cam_depth_color.png', d_color)
            self.get_logger().info(f'Saved: ~/cam_color.png, ~/cam_depth_raw.png, ~/cam_depth_color.png')
            self.get_logger().info('Selesai. Ctrl+C.')
            raise SystemExit

def main():
    rclpy.init()
    n = FrameGrabber()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
