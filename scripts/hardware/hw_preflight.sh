#!/usr/bin/env bash
# AAGV hardware preflight — READ ONLY. Tidak mengubah apa pun, tidak butuh sudo.
# Tujuan: cek apa yang dilihat PC sebelum bringup (port, USB, izin, USB3, paket ROS).
set +e
line(){ printf '\n=== %s ===\n' "$1"; }

line "1. Sistem & ROS"
. /etc/os-release 2>/dev/null; echo "OS    : $PRETTY_NAME"
echo "Kernel: $(uname -r)"
echo "ROS   : ${ROS_DISTRO:-(belum di-source)}  (harusnya jazzy)"

line "2. Serial port (OpenCR muncul sebagai /dev/ttyACM*)"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  (tidak ada ttyACM/ttyUSB — OpenCR belum terdeteksi?)"

line "3. Perangkat USB (cari OpenCR & RealSense)"
if command -v lsusb >/dev/null; then
  lsusb | grep -iE 'realsense|intel|0483|df11|stm|opencr' || echo "  (tak ada match nama; daftar lengkap di bawah)"
  echo "--- lsusb lengkap ---"; lsusb
else echo "  lsusb tidak ada: sudo apt install usbutils"; fi

line "4. OpenCR: VID 0483:5740=firmware OK | 0483:df11=mode DFU/bootloader"
lsusb 2>/dev/null | grep -iE '0483:5740|0483:df11|STMicro' || echo "  OpenCR tidak terlihat di VID 0483. Cek kabel USB & tombol power OpenCR."

line "5. RealSense: VID 8086 (D435i biasanya 8086:0b3a)"
lsusb 2>/dev/null | grep -iE '8086' || echo "  RealSense tidak terlihat (VID 8086). Cek kabel."

line "6. Cek USB 3.0 untuk RealSense (wajib 5000M)"
if command -v lsusb >/dev/null; then
  RS=$(lsusb -t 2>/dev/null | grep -iB1 'uvcvideo\|RealSense' )
  lsusb -t 2>/dev/null | grep -E '5000M|480M' | sed 's/^/  /'
  echo "  -> kalau perangkat Intel hanya di '480M' = itu USB2, depth bisa gagal. Harus 5000M."
fi

line "7. Izin akses serial (grup dialout)"
echo "User  : $USER"
echo "Groups: $(groups)"
groups | grep -qw dialout && echo "  OK: sudah di grup dialout" || echo "  BELUM di dialout -> nanti: sudo usermod -aG dialout $USER ; lalu logout/login"

line "8. Paket ROS terkait (sudah ada / belum)"
if command -v ros2 >/dev/null; then
  for p in open_manipulator_bringup open_manipulator_moveit_config dynamixel_hardware_interface realsense2_camera realsense2_description librealsense2; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "  [ada]   $p"; else echo "  [BELUM] $p"; fi
  done
else echo "  ros2 belum di-source. Jalankan: source /opt/ros/jazzy/setup.bash"; fi

line "9. Tool dynamixel (untuk ping servo nanti)"
for t in find_dynamixel; do
  if ros2 pkg executables dynamixel_sdk_custom_interfaces 2>/dev/null | grep -q . ; then :; fi
done
command -v rs-enumerate-devices >/dev/null && echo "  rs-enumerate-devices: ADA (tool realsense terpasang)" || echo "  rs-enumerate-devices: belum (akan terpasang bareng driver realsense)"

echo
echo "=== SELESAI. Salin SELURUH output ini ke chat. ==="
