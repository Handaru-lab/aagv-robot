#!/usr/bin/env python3
"""
unified_detector.py — Deteksi objek via webcam laptop (standalone, tanpa ROS).
3 mode deteksi, switchable via keyboard:
  [1] color  — HSV range (untuk bola pingpong berwarna)
  [2] dark   — grayscale threshold (untuk penghapus hitam di meja terang)
  [3] skin   — HSV range skin-tone (untuk tangan)

Kontrol:
  1/2/3  — ganti mode
  t      — toggle trackbar tuning window
  s      — simpan HSV range saat ini ke file
  c      — capture frame + mask ke file
  q/ESC  — quit

Jalankan:
  python3 unified_detector.py
  python3 unified_detector.py --camera 0        # pilih kamera (default 0)
  python3 unified_detector.py --mode color       # mulai di mode tertentu
"""
import cv2
import numpy as np
import argparse
import json
import os
from datetime import datetime

# ============================================================
# Default HSV ranges per mode
# ============================================================
PRESETS = {
    'color': {
        'label': 'Color (HSV)',
        'description': 'Bola pingpong berwarna — tune H range sesuai warna cat',
        # Default: oranye (dari sim). Tune via trackbar.
        'h_lo': 5, 'h_hi': 25,
        's_lo': 100, 's_hi': 255,
        'v_lo': 100, 'v_hi': 255,
        'min_area': 500,
        'color_bgr': (0, 140, 255),  # oranye untuk overlay
    },
    'dark': {
        'label': 'Dark Object (Gray)',
        'description': 'Objek gelap di meja terang — threshold grayscale',
        'h_lo': 0, 'h_hi': 180,
        's_lo': 0, 's_hi': 255,
        'v_lo': 0, 'v_hi': 90,   # pixel gelap = V rendah
        'min_area': 1500,
        'color_bgr': (100, 100, 100),
    },
    'skin': {
        'label': 'Skin Tone (HSV)',
        'description': 'Telapak tangan — HSV range kulit',
        'h_lo': 0, 'h_hi': 25,
        's_lo': 40, 's_hi': 180,
        'v_lo': 80, 'v_hi': 255,
        'min_area': 3000,
        'color_bgr': (0, 200, 200),
    },
}


class UnifiedDetector:
    def __init__(self, camera_id=0, mode='color'):
        self.camera_id = camera_id
        self.mode = mode
        self.show_trackbars = False
        self.trackbar_window = 'HSV Tuning'

        # Load saved presets if exist
        self.config_path = os.path.expanduser('~/detector_presets.json')
        self.load_presets()

        # Current working values (copy from preset)
        self.params = dict(PRESETS[self.mode])

    def load_presets(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path) as f:
                    saved = json.load(f)
                for mode_name, values in saved.items():
                    if mode_name in PRESETS:
                        PRESETS[mode_name].update(values)
                print(f"[INFO] Loaded presets from {self.config_path}")
            except Exception as e:
                print(f"[WARN] Could not load presets: {e}")

    def save_presets(self):
        save_data = {}
        for mode_name, preset in PRESETS.items():
            save_data[mode_name] = {
                k: v for k, v in preset.items()
                if k in ('h_lo', 'h_hi', 's_lo', 's_hi', 'v_lo', 'v_hi', 'min_area')
            }
        with open(self.config_path, 'w') as f:
            json.dump(save_data, f, indent=2)
        print(f"[INFO] Presets saved to {self.config_path}")

    def switch_mode(self, new_mode):
        # Save current trackbar values back to preset
        self._save_trackbar_to_preset()
        self.mode = new_mode
        self.params = dict(PRESETS[self.mode])
        if self.show_trackbars:
            self._update_trackbars()
        print(f"[MODE] Switched to: {self.params['label']} — {self.params['description']}")

    def _save_trackbar_to_preset(self):
        if self.show_trackbars:
            try:
                PRESETS[self.mode]['h_lo'] = cv2.getTrackbarPos('H Lo', self.trackbar_window)
                PRESETS[self.mode]['h_hi'] = cv2.getTrackbarPos('H Hi', self.trackbar_window)
                PRESETS[self.mode]['s_lo'] = cv2.getTrackbarPos('S Lo', self.trackbar_window)
                PRESETS[self.mode]['s_hi'] = cv2.getTrackbarPos('S Hi', self.trackbar_window)
                PRESETS[self.mode]['v_lo'] = cv2.getTrackbarPos('V Lo', self.trackbar_window)
                PRESETS[self.mode]['v_hi'] = cv2.getTrackbarPos('V Hi', self.trackbar_window)
                PRESETS[self.mode]['min_area'] = cv2.getTrackbarPos('Min Area', self.trackbar_window)
            except cv2.error:
                pass

    def _create_trackbars(self):
        cv2.namedWindow(self.trackbar_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.trackbar_window, 400, 300)
        noop = lambda x: None
        cv2.createTrackbar('H Lo', self.trackbar_window, self.params['h_lo'], 180, noop)
        cv2.createTrackbar('H Hi', self.trackbar_window, self.params['h_hi'], 180, noop)
        cv2.createTrackbar('S Lo', self.trackbar_window, self.params['s_lo'], 255, noop)
        cv2.createTrackbar('S Hi', self.trackbar_window, self.params['s_hi'], 255, noop)
        cv2.createTrackbar('V Lo', self.trackbar_window, self.params['v_lo'], 255, noop)
        cv2.createTrackbar('V Hi', self.trackbar_window, self.params['v_hi'], 255, noop)
        cv2.createTrackbar('Min Area', self.trackbar_window, self.params['min_area'], 20000, noop)

    def _update_trackbars(self):
        try:
            cv2.setTrackbarPos('H Lo', self.trackbar_window, self.params['h_lo'])
            cv2.setTrackbarPos('H Hi', self.trackbar_window, self.params['h_hi'])
            cv2.setTrackbarPos('S Lo', self.trackbar_window, self.params['s_lo'])
            cv2.setTrackbarPos('S Hi', self.trackbar_window, self.params['s_hi'])
            cv2.setTrackbarPos('V Lo', self.trackbar_window, self.params['v_lo'])
            cv2.setTrackbarPos('V Hi', self.trackbar_window, self.params['v_hi'])
            cv2.setTrackbarPos('Min Area', self.trackbar_window, self.params['min_area'])
        except cv2.error:
            self._create_trackbars()

    def _read_trackbars(self):
        try:
            self.params['h_lo'] = cv2.getTrackbarPos('H Lo', self.trackbar_window)
            self.params['h_hi'] = cv2.getTrackbarPos('H Hi', self.trackbar_window)
            self.params['s_lo'] = cv2.getTrackbarPos('S Lo', self.trackbar_window)
            self.params['s_hi'] = cv2.getTrackbarPos('S Hi', self.trackbar_window)
            self.params['v_lo'] = cv2.getTrackbarPos('V Lo', self.trackbar_window)
            self.params['v_hi'] = cv2.getTrackbarPos('V Hi', self.trackbar_window)
            self.params['min_area'] = max(1, cv2.getTrackbarPos('Min Area', self.trackbar_window))
        except cv2.error:
            pass

    def detect(self, frame):
        """Run detection on frame. Returns (annotated_frame, mask, detections)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if self.show_trackbars:
            self._read_trackbars()

        p = self.params

        if self.mode == 'dark':
            # Dark mode: use grayscale threshold (V channel only)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mask = (gray < p['v_hi']).astype(np.uint8) * 255
            # Also apply S filter if needed
            if p['s_hi'] < 255:
                s_mask = hsv[:, :, 1] < p['s_hi']
                mask = mask & (s_mask.astype(np.uint8) * 255)
        else:
            # HSV range threshold
            lo = np.array([p['h_lo'], p['s_lo'], p['v_lo']])
            hi = np.array([p['h_hi'], p['s_hi'], p['v_hi']])

            # Handle H wrap-around (e.g. red: h_lo=170, h_hi=10)
            if p['h_lo'] > p['h_hi']:
                mask1 = cv2.inRange(hsv, np.array([p['h_lo'], p['s_lo'], p['v_lo']]),
                                         np.array([180, p['s_hi'], p['v_hi']]))
                mask2 = cv2.inRange(hsv, np.array([0, p['s_lo'], p['v_lo']]),
                                         np.array([p['h_hi'], p['s_hi'], p['v_hi']]))
                mask = mask1 | mask2
            else:
                mask = cv2.inRange(hsv, lo, hi)

        # Morphology
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if cv2.contourArea(c) >= p['min_area']]

        # Sort by area descending
        big.sort(key=cv2.contourArea, reverse=True)

        vis = frame.copy()
        detections = []
        color = p['color_bgr']

        for i, cnt in enumerate(big[:5]):  # max 5 detections
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Bounding rect
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / max(h, 1)
            circularity = 4 * np.pi * area / max(cv2.arcLength(cnt, True) ** 2, 1)

            detections.append({
                'id': i,
                'cx': cx, 'cy': cy,
                'area': area,
                'aspect': aspect,
                'circularity': circularity,
                'bbox': (x, y, w, h),
            })

            # Draw
            cv2.drawContours(vis, [cnt], -1, color, 2)
            cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 1)

            # Info text
            info1 = f"#{i} A={area:.0f}"
            info2 = f"circ={circularity:.2f} asp={aspect:.2f}"
            cv2.putText(vis, info1, (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(vis, info2, (cx + 8, cy + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

        return vis, mask, detections

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f"[ERROR] Cannot open camera {self.camera_id}")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("=" * 60)
        print("  AAGV Unified Detector — Webcam Mode")
        print("=" * 60)
        print(f"  Camera: {self.camera_id}")
        print(f"  Mode  : {self.params['label']}")
        print()
        print("  Keys:")
        print("    1 = Color (bola pingpong)")
        print("    2 = Dark  (penghapus hitam)")
        print("    3 = Skin  (tangan)")
        print("    t = Toggle HSV trackbar tuning")
        print("    s = Save current presets")
        print("    c = Capture frame + mask")
        print("    q / ESC = Quit")
        print("=" * 60)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Frame capture failed")
                break

            vis, mask, detections = self.detect(frame)

            # HUD overlay
            H, W = vis.shape[:2]
            # Mode indicator
            mode_text = f"[{self.mode.upper()}] {self.params['label']}"
            cv2.rectangle(vis, (0, 0), (W, 30), (40, 40, 40), -1)
            cv2.putText(vis, mode_text, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            # Detection count
            det_text = f"{len(detections)} detected"
            cv2.putText(vis, det_text, (W - 140, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 1)

            # HSV range on screen
            p = self.params
            range_text = f"H[{p['h_lo']}-{p['h_hi']}] S[{p['s_lo']}-{p['s_hi']}] V[{p['v_lo']}-{p['v_hi']}] minA={p['min_area']}"
            cv2.putText(vis, range_text, (10, H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

            # Show
            cv2.imshow('Detector', vis)

            # Mask as color overlay
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.imshow('Mask', mask_color)

            # Key handling
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or ESC
                break
            elif key == ord('1'):
                self.switch_mode('color')
            elif key == ord('2'):
                self.switch_mode('dark')
            elif key == ord('3'):
                self.switch_mode('skin')
            elif key == ord('t'):
                self.show_trackbars = not self.show_trackbars
                if self.show_trackbars:
                    self._create_trackbars()
                    print("[INFO] Trackbar ON — drag sliders to tune HSV range")
                else:
                    cv2.destroyWindow(self.trackbar_window)
                    print("[INFO] Trackbar OFF")
            elif key == ord('s'):
                self._save_trackbar_to_preset()
                self.save_presets()
            elif key == ord('c'):
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                cv2.imwrite(os.path.expanduser(f'~/capture_{ts}_frame.png'), frame)
                cv2.imwrite(os.path.expanduser(f'~/capture_{ts}_mask.png'), mask)
                cv2.imwrite(os.path.expanduser(f'~/capture_{ts}_vis.png'), vis)
                print(f"[INFO] Captured: ~/capture_{ts}_*.png")

        self._save_trackbar_to_preset()
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Detector stopped.")


def main():
    parser = argparse.ArgumentParser(description='AAGV Unified Detector (webcam)')
    parser.add_argument('--camera', type=int, default=0, help='Camera ID (default: 0)')
    parser.add_argument('--mode', choices=['color', 'dark', 'skin'], default='color',
                        help='Starting detection mode')
    args = parser.parse_args()

    detector = UnifiedDetector(camera_id=args.camera, mode=args.mode)
    detector.run()


if __name__ == '__main__':
    main()
