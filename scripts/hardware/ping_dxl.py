#!/usr/bin/env python3
"""AAGV - ping servo OMX (READ ONLY). Tidak menyalakan torque, tidak menggerakkan.
Ping ID 11-15 di /dev/ttyACM0 @ 1Mbps, baca model + posisi sekarang."""
import sys
try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("dynamixel_sdk tidak ada. Jalankan:\n  pip install dynamixel-sdk --break-system-packages")
    sys.exit(1)

PORT = "/dev/ttyACM0"
BAUD = 1000000
IDS  = [11, 12, 13, 14, 15]          # 11-14 joint, 15 gripper
ADDR_PRESENT_POSITION = 132          # XM430 Protocol 2.0, 4 byte

ph = PortHandler(PORT)
pk = PacketHandler(2.0)

if not ph.openPort():
    print(f"GAGAL buka {PORT}. Cek: kabel, grup dialout, dan tidak ada proses lain memakai port (mis. launch).")
    sys.exit(1)
if not ph.setBaudRate(BAUD):
    print("GAGAL set baudrate 1000000."); sys.exit(1)

print(f"Port {PORT} @ {BAUD} terbuka. Ping ID {IDS}...\n")
print(f"{'ID':<4}{'status':<10}{'model':<8}{'pos(raw)':<10}{'pos(deg)':<10}")
ok = 0
for i in IDS:
    model, comm, err = pk.ping(ph, i)
    if comm != 0:
        print(f"{i:<4}{'NO RESP':<10}{'-':<8}{'-':<10}{'-':<10}")
        continue
    if err != 0:
        print(f"{i:<4}{'ERR '+str(err):<10}{model:<8}{'-':<10}{'-':<10}")
        continue
    pos, c2, e2 = pk.read4ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
    if pos > 0x7fffffff:
        pos -= 0x100000000
    deg = pos * 360.0 / 4096.0
    print(f"{i:<4}{'OK':<10}{model:<8}{pos:<10}{deg:<10.1f}")
    ok += 1

ph.closePort()
print(f"\n{ok}/{len(IDS)} servo merespons.")
if ok == len(IDS):
    print("SEMUA servo hidup -> OpenCR bridge OK, siap launch hardware.")
    print("Cek: pos(deg) tiap joint masuk akal (lengan tidak terpuntir ekstrem) sebelum torque-on.")
elif ok == 0:
    print("TIDAK ada yang merespons -> kemungkinan firmware OpenCR bukan passthrough, atau power servo OFF.")
else:
    print("Sebagian tidak merespons -> cek daisy-chain/konektor servo yang gagal di atas.")
