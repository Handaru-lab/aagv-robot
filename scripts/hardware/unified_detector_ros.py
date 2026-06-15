#!/usr/bin/env python3
"""
unified_detector_ros.py — Unified detector untuk RealSense di ROS2.
Konfigurasi HSV sama persis dengan unified_detector.py (webcam).
Preset di-load dari ~/detector_presets.json (hasil tuning di webcam).

3 mode (ganti via ROS param 'detection_mode'):
  color  — HSV range (bola pingpong berwarna)
  dark   — grayscale threshold (penghapus hitam)
  skin   — HSV range skin-tone (tangan)

Topik subscribe:
  /camera/camera/color/image_raw
  /camera/camera/depth/image_rect_raw
  /camera/camera/color/camera_info

Topik publish:
  /object/point       (PointStamped)    — posisi 3D di camera_color_optical_frame
  /object/marker      (Marker)          — sphere marker untuk RViz
  /object/mask        (Image, mono8)    — mask debug
  /object/annotated   (Image, bgr8)     — frame dengan overlay deteksi

Jalankan:
  python3 unified_detector_ros.py
  python3 unified_detector_ros.py --ros-args -p detection_mode:=dark
  python3 unified_detector_ros.py --ros-args -p detection_mode:=skin
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
import json
import os
import message_filters

# ============================================================
# Default presets (identical to unified_detector.py webcam)
# ============================================================
PRESETS = {
    'color': {
        'h_lo': 5, 'h_hi': 25,
        's_lo': 100, 's_hi': 255,
        'v_lo': 100, 'v_hi': 255,
        'min_area': 500,
        'color_bgr': (0, 140, 255),
    },
    'dark': {
        'h_lo': 0, 'h_hi': 180,
        's_lo': 0, 's_hi': 255,
        'v_lo': 0, 'v_hi': 90,
        'min_area': 1500,
        'color_bgr': (100, 100, 100),
    },
    'skin': {
        'h_lo': 0, 'h_hi': 25,
        's_lo': 40, 's_hi': 180,
        'v_lo': 80, 'v_hi': 255,
        'min_area': 3000,
        'color_bgr': (0, 200, 200),
    },
}

# Object height per mode (for depth ring fallback on IR-absorbing surfaces)
OBJECT_HEIGHT = {
    'color': 0.02,   # bola pingpong ~2cm radius
    'dark': 0.04,    # penghapus ~4cm
    'skin': 0.02,    # tangan ~2cm above table
}


def load_presets():
    """Load saved presets from webcam tuning session."""
    path = os.path.expanduser('~/detector_presets.json')
    if os.path.exists(path):
        try:
            with open(path) as f:
                saved = json.load(f)
            for mode_name, values in saved.items():
                if mode_name in PRESETS:
                    PRESETS[mode_name].update(values)
            return True, path
        except Exception:
            return False, path
    return False, path


class UnifiedDetectorROS(Node):
    def __init__(self):
        super().__init__('unified_detector')

        # Load presets from webcam tuning
        loaded, path = load_presets()
        if loaded:
            self.get_logger().info(f'Loaded presets from {path}')
        else:
            self.get_logger().warn(f'No presets at {path}, using defaults. Run unified_detector.py (webcam) to tune.')

        # Parameters
        self.declare_parameter('detection_mode', 'color')
        self.declare_parameter('depth_ring_dilate', 40)
        self.declare_parameter('roi_top_fraction', 0.0)
        self.declare_parameter('publish_interval', 1.0)  # seconds, 0=every frame
        self._last_publish_time = 0.0

        self.br = CvBridge()
        self.intrinsics = None

        # Subscribers
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.cb_info, 10)

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

        mode = self.get_parameter('detection_mode').value
        self.get_logger().info(f'UnifiedDetectorROS started — mode: {mode}')
        self.get_logger().info(f'  Preset: H[{PRESETS[mode]["h_lo"]}-{PRESETS[mode]["h_hi"]}] '
                               f'S[{PRESETS[mode]["s_lo"]}-{PRESETS[mode]["s_hi"]}] '
                               f'V[{PRESETS[mode]["v_lo"]}-{PRESETS[mode]["v_hi"]}] '
                               f'minA={PRESETS[mode]["min_area"]}')

    def cb_info(self, msg):
        if self.intrinsics is None:
            K = msg.k
            self.intrinsics = (K[0], K[4], K[2], K[5])
            self.get_logger().info(
                f'Camera intrinsics: fx={K[0]:.1f} fy={K[4]:.1f} cx={K[2]:.1f} cy={K[5]:.1f}')

    def make_mask(self, frame, mode):
        """Generate detection mask based on mode."""
        p = PRESETS[mode]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, W = frame.shape[:2]

        # ROI: mask out top fraction
        roi_top = self.get_parameter('roi_top_fraction').value
        roi_y = int(H * roi_top)

        if mode == 'dark':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mask = (gray < p['v_hi']).astype(np.uint8) * 255
        else:
            lo = np.array([p['h_lo'], p['s_lo'], p['v_lo']])
            hi = np.array([p['h_hi'], p['s_hi'], p['v_hi']])

            if p['h_lo'] > p['h_hi']:
                mask1 = cv2.inRange(hsv,
                    np.array([p['h_lo'], p['s_lo'], p['v_lo']]),
                    np.array([180, p['s_hi'], p['v_hi']]))
                mask2 = cv2.inRange(hsv,
                    np.array([0, p['s_lo'], p['v_lo']]),
                    np.array([p['h_hi'], p['s_hi'], p['v_hi']]))
                mask = mask1 | mask2
            else:
                mask = cv2.inRange(hsv, lo, hi)

        # Blank top ROI
        if roi_y > 0:
            mask[:roi_y, :] = 0

        # Morphology
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        return mask

    def get_depth_at_contour(self, depth, target, cx, cy, mode):
        """Get depth: try centroid first, fallback to ring around contour."""
        H, W = depth.shape[:2]
        margin = 5
        obj_height = OBJECT_HEIGHT.get(mode, 0.03)

        # Try centroid directly
        y_lo = max(0, cy - margin)
        y_hi = min(H, cy + margin + 1)
        x_lo = max(0, cx - margin)
        x_hi = min(W, cx + margin + 1)
        patch = depth[y_lo:y_hi, x_lo:x_hi]
        valid = patch[patch > 0]

        if len(valid) > 0:
            return float(np.median(valid)) / 1000.0, 'centroid'

        # Fallback: ring around contour (table surface)
        dilate_px = self.get_parameter('depth_ring_dilate').value
        obj_mask = np.zeros((H, W), dtype=np.uint8)
        cv2.drawContours(obj_mask, [target], -1, 255, -1)
        dilate_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px, dilate_px))
        dilated = cv2.dilate(obj_mask, dilate_k)
        ring_mask = dilated & (~obj_mask)
        ring_depth = depth[ring_mask > 0]
        valid_ring = ring_depth[ring_depth > 0]

        if len(valid_ring) == 0:
            return None, 'no_depth'

        table_z_m = float(np.median(valid_ring)) / 1000.0
        obj_z_m = table_z_m - obj_height
        return obj_z_m, 'ring'

    def cb_frames(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return

        mode = self.get_parameter('detection_mode').value
        if mode not in PRESETS:
            self.get_logger().error(f'Unknown mode: {mode}', throttle_duration_sec=5.0)
            return

        p = PRESETS[mode]
        fx, fy, cx_cam, cy_cam = self.intrinsics

        color = self.br.imgmsg_to_cv2(color_msg, 'bgr8')
        depth = self.br.imgmsg_to_cv2(depth_msg, 'passthrough')
        H, W = color.shape[:2]

        # Detect
        mask = self.make_mask(color, mode)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if cv2.contourArea(c) >= p['min_area']]

        # Publish mask
        self.pub_mask.publish(self.br.cv2_to_imgmsg(mask, 'mono8'))

        if not big:
            return

        import time as _time
        _now = _time.time()
        _interval = self.get_parameter('publish_interval').value
        if _interval > 0 and (_now - self._last_publish_time) < _interval:
            # Still publish mask even when throttled
            return
        self._last_publish_time = _now

        # Largest contour = target
        target = max(big, key=cv2.contourArea)
        M = cv2.moments(target)
        if M['m00'] == 0:
            return
        px = int(M['m10'] / M['m00'])
        py = int(M['m01'] / M['m00'])

        # Depth
        z_m, depth_source = self.get_depth_at_contour(depth, target, px, py, mode)
        if z_m is None:
            self.get_logger().warn('No valid depth', throttle_duration_sec=2.0)
            return

        if depth_source == 'ring':
            self.get_logger().info(
                f'Depth from ring (table): z={z_m:.3f}m', throttle_duration_sec=5.0)

        # Deproject
        x_m = (px - cx_cam) * z_m / fx
        y_m = (py - cy_cam) * z_m / fy

        # Publish PointStamped
        pt = PointStamped()
        pt.header = color_msg.header
        pt.point.x = x_m
        pt.point.y = y_m
        pt.point.z = z_m
        self.pub_point.publish(pt)

        # Publish Marker
        mk = Marker()
        mk.header = color_msg.header
        mk.ns = 'object'
        mk.id = 0
        mk.type = Marker.SPHERE
        mk.action = Marker.ADD
        mk.pose.position.x = x_m
        mk.pose.position.y = y_m
        mk.pose.position.z = z_m
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.04
        color_bgr = p['color_bgr']
        mk.color.r = color_bgr[2] / 255.0
        mk.color.g = color_bgr[1] / 255.0
        mk.color.b = color_bgr[0] / 255.0
        mk.color.a = 0.9
        mk.lifetime.sec = 0
        mk.lifetime.nanosec = 300_000_000
        self.pub_marker.publish(mk)

        # Publish annotated
        vis = color.copy()
        draw_color = p['color_bgr']
        cv2.drawContours(vis, [target], -1, draw_color, 2)
        cv2.circle(vis, (px, py), 5, (0, 0, 255), -1)
        area = cv2.contourArea(target)
        circ = 4 * np.pi * area / max(cv2.arcLength(target, True) ** 2, 1)
        label1 = f'({x_m:.3f}, {y_m:.3f}, {z_m:.3f})m'
        label2 = f'A={area:.0f} circ={circ:.2f} [{depth_source}]'
        cv2.putText(vis, label1, (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(vis, label2, (px + 10, py + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, draw_color, 1)

        # Mode HUD
        mode_text = f'[{mode.upper()}] H[{p["h_lo"]}-{p["h_hi"]}] S[{p["s_lo"]}-{p["s_hi"]}] V[{p["v_lo"]}-{p["v_hi"]}]'
        cv2.putText(vis, mode_text, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        self.pub_annotated.publish(self.br.cv2_to_imgmsg(vis, 'bgr8'))


def main():
    rclpy.init()
    node = UnifiedDetectorROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
