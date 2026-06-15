# AAGV Robot — Session Summary
## 14-15 Juni 2026

### Ringkasan

Sesi ini mencakup dua fase besar: (1) penyelesaian dokumen UAS Metode Numerik, dan (2) hardware bringup lengkap OpenManipulator-X + RealSense D435i + MoveIt, termasuk pembuatan autonomous scan-pick-place pipeline.

---

### Fase 1: UAS Metode Numerik (Selesai)

Membuat dokumen analisis korelasi mata kuliah Metode Numerik dengan proyek AAGV. Dokumen 8 halaman (Word + PDF) berisi:

- Bagian A: Checklist 9 topik metode numerik yang berkorelasi dengan proyek (8 relevan, 1 skip)
- Soal 1: Deep dive dua topik — Sistem Persamaan Linear (homogeneous transform, R²≈0.9999) dan Interpolasi (spline trajectory controller)
- Bagian B: Pemetaan fase proyek ke topik metode numerik
- Eksperimen 1: Perception accuracy (13 titik, RMSE 2.6cm, MAPE 3.6%)
- Eksperimen 2: Trajectory analysis (posisi, kecepatan, integrasi, diferensiasi)
- **Repeatability (ditambahkan sesi ini)**: Perception 10× (std=0, galat 100% sistematis) dan trajectory 10× (integrasi CV 0.03%, diferensiasi CV 5.6%)
- Validasi dan kesimpulan

File: `Analisis_Korelasi_Metode_Numerik.docx` dan `.pdf`

---

### Fase 2: Hardware Bringup (Fase A–D)

#### Fase A — Preflight (Selesai ✅)
- Script `hw_preflight.sh` mengecek USB devices, serial ports, permissions, ROS packages
- Hasil: OpenCR terdeteksi (`0483:5740` = firmware OK), RealSense D435i terdeteksi (`8086:0b3a`)
- **Issue ditemukan**: User belum di grup `dialout` → di-fix via `usermod`
- **Issue ditemukan**: RealSense di USB 2.1 (bukan USB 3.0) — semua port laptop 480M, tidak ada 5000M. Diterima sebagai limitasi (depth tetap berfungsi di USB2)

#### Fase B — Arm Bringup (Selesai ✅)
- Script `ping_dxl.py` — ping 5/5 servo (ID 11-15, model 1020 XM430-W350) sukses
- Launch `open_manipulator_x.launch.py port_name:=/dev/ttyACM0` — semua controller aktif (arm_controller, gripper_controller, joint_state_broadcaster)
- `joint_trajectory_executor` → `Goal reached, success!`
- **MoveIt pada hardware nyata** berhasil: Plan & Execute via RViz → `SUCCEEDED`

#### Fase C — RealSense D435i (Selesai ✅)
- Install `ros-jazzy-realsense2-camera`
- Launch `rs_launch.py` dengan profil 640×480@15fps
- Color + depth streaming stabil ~15Hz
- Warning `Depth stream start failure` transient — depth recover sendiri
- Kernel 6.17 > DKMS support (max 6.14) — pakai RS-USB backend, tanpa patch kernel

#### Fase D — Integrasi (In Progress)
- Static TF publisher: `link5 → camera_link` (extrinsic dari sim: xyz 0.072 0 0.04)
- TF bridge berhasil: `world → camera_color_optical_frame` terhubung

---

### Perception — Dark Object Detector

Penghapus hitam dipakai sebagai objek tes (bola pingpong belum tersedia).

**Masalah yang diatasi:**
1. HSV ball detector dari sim tidak berlaku untuk objek hitam → buat detector baru berbasis grayscale threshold
2. Analisis histogram: eraser body gray mean=88, table mean=127 → threshold ~90-95 (Otsu) berhasil isolasi 1 kontur
3. **Depth = 0 pada objek hitam** — IR structured light diserap permukaan hitam matte → solusi: sample depth dari ring di sekitar kontur (meja kayu = depth valid), kurangi tinggi objek (4cm)
4. **TF timestamp extrapolation** — fix dengan `Time(seconds=0)` untuk latest available transform

**Hasil**: Deteksi berhasil, objek di world frame `(0.207, 0.003, 0.039)` — 20.7cm di depan base, masuk akal.

---

### Pick Demo

#### hw_pick.py (v1) — Position-only MoveIt
- **Berhasil pick 2/3 kali** (penghapus posisi tidur)
- Grip loyo karena grasp height terlalu tinggi (`obj_z + 0.02`)
- Posisi berdiri gagal semua (gripper aperture < penghapus width)

#### hw_pick_v2.py — Pose goal (position + orientation)
- Menambahkan orientation constraint (gripper down: quaternion Ry(-90°))
- Belum berhasil dijalankan karena beralih ke MoveIt Setup Assistant

---

### MoveIt Setup Assistant (aagv_moveit_config)

- Launch via `QT_QPA_PLATFORM=xcb` (fix Wayland crash)
- Input: URDF AAGV (`/tmp/aagv.urdf` dari xacro)
- Output: package `aagv_moveit_config` di `~/aagv_ws/src/`
- Planning groups: `arm` (joint1-4) dan `gripper`
- Named states: `init`, `home`, `open`, `close`
- Collision matrix termasuk `camera_link` dan `base_link`
- 4 planner: OMPL, CHOMP, Pilz (LIN/PTP/CIRC), STOMP

**Fixes yang diperlukan setelah generate:**
1. `moveit_controllers.yaml` — tambah `action_ns` (missing) dan fix gripper type ke `GripperCommand`
2. `joint_limits.yaml` — tambah `has_acceleration_limits: true` dan `max_acceleration: 5.0` (missing, menyebabkan `AddTimeOptimalParameterization` gagal)
3. URDF `base_link` collision box di-remove — menghalangi MoveIt planning ke target low-z (AGV box fisik tidak ada di setup hardware)

---

### Unified Detector

Dibuat dua versi:

1. **`unified_detector.py`** (standalone, webcam laptop) — untuk tuning HSV range tanpa ROS/hardware. 3 mode (color/dark/skin), trackbar tuning, save presets ke `~/detector_presets.json`.

2. **`unified_detector_ros.py`** (ROS2 node, RealSense) — load presets dari file yang sama, publish `/object/point`, `/object/marker`, `/object/mask`, `/object/annotated`. Parameter `detection_mode` switchable tanpa restart. Throttle via `publish_interval`.

**Tes berhasil:**
- Mode `dark`: penghapus terdeteksi, depth dari ring ~0.19m, publish rate 1Hz (throttled) ✅
- Mode `skin`: tangan terdeteksi, depth ~0.27m ✅
- Mode `color`: menunggu bola pingpong berwarna

---

### Autonomous Scan-Pick-Place (auto_pick_place.py)

**Arsitektur:**
- Scan: 7 arah compass × 10 level vertikal = 70 poses max
- Start pose: j2=-75°, j3=+75°, j4=0° (kamera horizontal, lengan bent)
- Tiap level: j4 bertambah +10° (wrist menunduk, positif = down di MoveIt)
- Early exit: stop scan begitu objek terdeteksi dalam reach
- Pick: open → approach → descend → close → lift
- Place: move ke koordinat tetap → release
- Home: kembali ke scan start pose

**Status saat ini:**
- Scan berhasil — objek ditemukan di pose pertama (N_L0, horizontal) ✅
- Deteksi: `(0.334, 0.079, 0.131)` di link1 frame, r=0.343m
- **Approach gagal** (ec=99999) — target r=0.343m terlalu dekat ke batas reach, approach +6cm z membuat IK infeasible
- Fix pending: kurangi reach threshold ke 0.30m dan approach z ke +4cm

**Issue AAGV MoveIt config vs OMX standar:**
- AAGV URDF punya `arm_mount_z=0.25` → offset semua koordinat world frame
- Solusi: planning di `link1` frame (hw_pick_v3.py) menghindari offset
- `CONTROL_FAILED` / timestamp mismatch terjadi saat MoveIt dan arm bringup tidak di-restart bersamaan

---

### Git Repository

Repo: `github.com/Handaru-lab/aagv-robot` (branch `main`)

**Commits sesi ini:**
1. `feat: pick-place demo (MoveIt grasp + attach + place) + perception data scripts` (sebelum sesi)
2. `feat: aagv_moveit_config (Setup Assistant) + hardware scripts`
3. `docs: update README — full project status, hardware bringup, repo layout`
4. `fix: remove base_link collision box — unblocks MoveIt planning`
5. `feat: unified detector — webcam tuner + ROS2 node (color/dark/skin, throttle, shared presets)`

**Struktur repo saat ini:**
```
aagv-robot/
├── aagv_description/       # URDF, Gazebo worlds, launch
├── aagv_perception/        # Ball detector (sim)
├── aagv_moveit_config/     # MoveIt config (Setup Assistant)
├── scripts/hardware/       # 8 standalone scripts
│   ├── hw_preflight.sh
│   ├── ping_dxl.py
│   ├── grab_frame.py
│   ├── dark_object_detector.py
│   ├── unified_detector.py
│   ├── unified_detector_ros.py
│   ├── hw_pick.py / hw_pick_v2.py / hw_pick_v3.py
│   └── auto_pick_place.py
├── docs/
└── README.md
```

---

### TODO / Next Steps

1. **Fix approach reach** — kurangi threshold ke 0.30m, approach z ke +4cm
2. **Tes auto_pick_place end-to-end** — scan → pick → place → home
3. **Tune HSV untuk bola pingpong** — saat bola tersedia, tune via `unified_detector.py` (webcam)
4. **Hand-eye calibration** — extrinsic dari sim punya ~2-3cm offset, perlu `easy_handeye2` untuk presisi
5. **Place ke bin** — upgrade dari titik tetap ke bin berbeda per warna objek
6. **Timer trigger** — integrasi jadwal alarm untuk autonomous cycle
7. **Reeman AGV integration** — belum dimulai
