# AAGV Robot — Setup & Progress Log

**Project:** Agriculture AGV (Reeman AGV + OpenManipulator-X)
**Environment:** Ubuntu 24.04 LTS · ROS 2 Jazzy Jalisco · MoveIt 2
**Workspace:** `~/aagv_ws`
**Machine:** `awmc@awmc-000`
**Last updated:** 2026-06-14

---

## Session summary

Started from a **fresh reinstall** of Ubuntu 24.04 (chosen over an in-place "reset"
because Ubuntu has no safe factory-reset and a scripted purge on a ROS-heavy system
risks an unbootable desktop). On the clean system we installed Terminator, a full
ROS 2 Jazzy desktop, created the `aagv_ws` workspace, and defined a **hybrid package
strategy** (apt for released deps, source only for what isn't released).

---

## 1. Base system + Terminator

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install terminator -y
```

## 2. Locale (UTF-8, required by ROS 2)

```bash
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 3. Repositories

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# ROS 2 apt source (current method — ros2-apt-source .deb, not manual GPG key)
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## 4. ROS 2 Jazzy Desktop (full) + dev tools

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y      # core + RViz + Gazebo + demos
sudo apt install ros-dev-tools -y          # colcon, rosdep, etc.
```

## 5. Environment

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# Optional, if multiple ROS machines share a network:
# echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init
rosdep update
```

**Smoke test (passed):** `ros2 run demo_nodes_cpp talker` + `ros2 run demo_nodes_py listener`.

---

## 6. Workspace `aagv_ws`

```bash
mkdir -p ~/aagv_ws/src
cd ~/aagv_ws
colcon build --symlink-install        # empty build to form build/ install/ log/
echo "source ~/aagv_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> Sourcing order in `~/.bashrc` matters: underlay `/opt/ros/jazzy/setup.bash` **before**
> overlay `~/aagv_ws/install/setup.bash`. Current order is correct.

---

## 7. Package strategy — HYBRID (apt + source)

Rule of thumb: **apt for anything you don't modify** (auto deps, updates with the
system, tested). **Source only for packages you develop or that aren't released**
to the Jazzy apt repo. Don't clone into `src/` what you could apt-install.

| Component | Method | Why |
|---|---|---|
| MoveIt 2 | apt (`ros-jazzy-moveit`) | Released, stable, not modified |
| dynamixel-sdk | apt (`ros-jazzy-dynamixel-sdk`) | Released |
| ros2_control + controllers | apt | Released |
| OpenManipulator-X | **source** (`ROBOTIS-GIT/open_manipulator`, branch `jazzy`) | **Not released for Jazzy in apt** — must clone |
| Reeman AGV driver | source / vendor SDK | Vendor robot, never in apt |
| Custom bringup (`aagv_bringup`, …) | source (in `src/`) | Your own code |

### Commands

```bash
# A. apt — core deps
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit ros-jazzy-dynamixel-sdk \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers ros-jazzy-position-controllers

# B. source — OpenManipulator-X (pinned to jazzy branch)
cd ~/aagv_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git

# C. resolve deps
cd ~/aagv_ws
rosdep install --from-paths src --ignore-src -r -y

# D. build
colcon build --symlink-install
source ~/aagv_ws/install/setup.bash
```

> Compatibility note: the official OpenManipulator repo was recently restructured into
> a unified package, which has caused breakage on `main`. Stay on the `jazzy` branch
> (or a stable release tag). If the build complains about a missing sibling ROBOTIS
> package (e.g. `dynamixel_hardware_interface`), clone that sibling too.
> Alternative for pure `ros2_control`: `dynamixel-community/dynamixel_hardware` (branch `jazzy`).

---

## 8. Next steps / TODO

- [ ] Run hybrid commands (section 7) and confirm `colcon build` is green.
- [ ] Add Reeman AGV driver source (need repo/SDK link).
- [ ] Configure Dynamixel USB latency (16ms → 1ms) once hardware is connected.
- [ ] Create custom `aagv_bringup` package in `src/`.
- [ ] Set up a `mock_components` / Gazebo sandbox **before** driving real servos.
- [ ] Revisit the old `joint_trajectory_controller` YAML issue with the new clean config.

---

*This log is a manual record of setup steps for reproducibility and progress review.*

## Update 2026-06-14 — Simulation sandbox validated
- [x] Gazebo (`open_manipulator_x_gazebo`) + MoveIt 2 (`open_manipulator_x_moveit`) bring-up
- [x] Plan & Execute SUCCEEDED — full chain MoveIt 2 -> arm_controller -> Gazebo
- Gotcha: MoveIt launch needs `use_sim:=true` (NOT `use_sim_time`) to sync clock with Gazebo.
  Without it, planning works but execution always CONTROL_FAILED (wall-clock vs sim-time mismatch).
- Order: launch Gazebo first (publishes /robot_description + /clock), then MoveIt.

## Hardware notes — OpenCR 1.0 (pending: unit not on hand)
- Controller board: **OpenCR 1.0** (NOT U2D2). Appears as `/dev/ttyACM0` (USB CDC).
- Bring-up: `ros2 launch open_manipulator_bringup open_manipulator_x.launch.py port_name:=/dev/ttyACM0`
- OpenCR requires firmware flashed (Arduino IDE + OpenCR board package) — verify correct firmware vs current Jazzy stack when unit is available.
- Permissions: `sudo usermod -aG dialout $USER` then re-login.
- NOTE: U2D2 FTDI latency tweak (16ms->1ms) does NOT apply to OpenCR (ttyACM/CDC).
- Pre-torque: set arm to recommended startup pose; joints must be within operable range.
- Have physical e-stop reachable for first hardware test.

## Update 2026-06-14 — aagv_description (subsystem #1) done
- [x] Package aagv_description: OpenManipulator-X (arm macro) + AGV base_link root + D435i eye-in-hand on link5.
- Camera mount (link5 frame): cam_xyz="0.072 0.0 0.04", cam_rpy="0 0 0". Lens (+X) faces grasp direction.
- Verified in RViz: camera frames follow link5 when joints jog → eye-in-hand TF chain correct.
- Note: mount value is nominal for sim; real value comes from hand-eye calibration on hardware.

## Update 2026-06-14 — aagv_description (subsystem #1) done
- [x] Package aagv_description: OpenManipulator-X (arm macro) + AGV base_link root + D435i eye-in-hand on link5.
- Camera mount (link5 frame): cam_xyz="0.072 0.0 0.04", cam_rpy="0 0 0". Lens (+X) faces grasp direction.
- Verified in RViz: camera frames follow link5 when joints jog → eye-in-hand TF chain correct.
- Note: mount value is nominal for sim; real value comes from hand-eye calibration on hardware.
