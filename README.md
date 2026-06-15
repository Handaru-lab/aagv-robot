# Agriculture AGV Robot

Agriculture AGV — an autonomous ground vehicle integrating a **Reeman AGV** base with an
**OpenManipulator-X** robotic arm and **Intel RealSense D435i** camera for vision-guided
pick-and-place in agricultural settings.

## System Overview

| Layer | Component |
|---|---|
| OS | Ubuntu 24.04 LTS (kernel 6.17) |
| Middleware | ROS 2 Jazzy Jalisco |
| Motion planning | MoveIt 2 (OMPL, CHOMP, Pilz, STOMP) |
| Arm | OpenManipulator-X — 4-DOF, XM430-W350 servos (ID 11-15) |
| Camera | Intel RealSense D435i (eye-in-hand, mounted on link5) |
| Controller board | OpenCR 1.0 (USB-to-Dynamixel bridge, `/dev/ttyACM0`) |
| Base | Reeman AGV (planned integration) |
| Actuation | Dynamixel Protocol 2.0 via OpenCR passthrough |
| Workspace | `~/aagv_ws` (symlinked from this repo) |

## Repository Layout

```
aagv-robot/
├── README.md
├── aagv_description/          # URDF/xacro, Gazebo worlds, launch files
│   ├── urdf/aagv.urdf.xacro   #   world → base_link → arm → D435i
│   ├── config/                 #   gz_bridge, controller manager YAML
│   ├── launch/                 #   gazebo.launch.py, display.launch.py
│   └── worlds/                 #   empty_sensors.sdf
├── aagv_perception/            # Vision pipeline (ball/object detection)
│   ├── aagv_perception/        #   ball_detector.py (HSV, sim)
│   └── launch/                 #   perception.launch.py
├── aagv_moveit_config/         # MoveIt config (from Setup Assistant)
│   ├── config/                 #   SRDF, kinematics, joint_limits, controllers
│   └── launch/                 #   move_group, RViz, demo
├── scripts/
│   └── hardware/               # Standalone hardware scripts
│       ├── hw_preflight.sh     #   USB/port/permission preflight check
│       ├── ping_dxl.py         #   Dynamixel ping (read-only, no torque)
│       ├── grab_frame.py       #   Capture color+depth frame from RealSense
│       ├── dark_object_detector.py  # Real-world object detector (grayscale threshold)
│       ├── hw_pick.py          #   Pick orchestrator v1 (position-only MoveIt)
│       └── hw_pick_v2.py       #   Pick orchestrator v2 (Pose goal + orientation)
└── docs/
    └── SETUP_LOG.md
```

## What Works

### Simulation (Gazebo Harmonic)
- Full URDF: AGV base + OMX arm + D435i eye-in-hand camera
- gz_ros2_control with arm_controller (JointTrajectoryController), gripper_controller, joint_state_broadcaster — all active
- MoveIt 2 plan and execute in sim (position-only IK, KDL solver)
- Ball perception: HSV color detection → depth deproject → tf2 transform to base_link
- Grasp attach/detach via Gazebo DetachableJoint plugin
- Pick-place demo orchestrator (known position → MoveIt → attach → lift → place)
- Perception accuracy: RMSE 2.6 cm, R² = 0.9999 (13 test points)

### Hardware (validated on physical OMX + OpenCR 1.0 + D435i)
- **Arm bringup**: 5/5 Dynamixel servos (XM430-W350, model 1020) ping OK, torque ON, all controllers active
- **MoveIt on real arm**: Plan and execute via RViz and programmatic API — working
- **RealSense D435i**: Color + depth streaming at 640×480 @ 15 fps via USB 2.1
- **Dark object detection**: Grayscale threshold detector for black objects on light table — depth sampled from surrounding ring (black surfaces absorb IR structured light)
- **Pick demo**: hw_pick.py successfully picked a whiteboard eraser (lying flat) in 2/3 trials

### MoveIt Configuration (aagv_moveit_config)
- Generated via MoveIt Setup Assistant with AAGV URDF
- Planning groups: `arm` (joint1-4) and `gripper` (gripper_left/right_joint)
- Named states: `init`, `home`, `open`, `close`
- Collision matrix includes camera_link and base_link
- Controllers: FollowJointTrajectory (arm) + GripperCommand (gripper)
- 4 planners available: OMPL (RRTConnect), CHOMP, Pilz (LIN/PTP/CIRC), STOMP

## Known Issues and TODO

- **AGV base collision vs hardware**: URDF has a collision box for the AGV base. On the physical test setup (arm on table, no AGV), this blocks MoveIt planning to low-z targets. Workaround: use OMX-standard MoveIt config for hardware, or remove base collision in URDF.
- **USB 3.0**: D435i runs at USB 2.1 on the current machine — depth works but at reduced bandwidth. Not a blocker for static object detection.
- **Eye-in-hand calibration**: Using sim values (xyz 0.072 0 0.04) as starting point. No formal hand-eye calibration yet. Pick accuracy has ~2-3 cm offset.
- **Scan pose geometry**: Camera coaxial with gripper on link5. Reachable zone conflicts with imageable distance at zero pose. Needs angled camera mount or dedicated scan pose.
- **Reeman AGV integration**: Not yet started.
- **Ball detection (real hardware)**: HSV ball detector works in sim only. Real-world uses dark_object_detector.py (grayscale threshold). Color ball detector for real hardware is pending.

## Quick Start

### Hardware Bringup

```bash
# 1. Preflight check (read-only)
bash scripts/hardware/hw_preflight.sh

# 2. Ping servos (read-only, no torque)
python3 scripts/hardware/ping_dxl.py

# 3. Set arm to init pose MANUALLY, then launch
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py port_name:=/dev/ttyACM0

# 4. Launch RealSense
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15 \
  enable_color:=true enable_depth:=true pointcloud.enable:=false

# 5. TF bridge (link5 → camera_link)
ros2 run tf2_ros static_transform_publisher \
  --x 0.072 --y 0.0 --z 0.04 --roll 0 --pitch 0 --yaw 0 \
  --frame-id link5 --child-frame-id camera_link

# 6. MoveIt (use OMX standard config for hardware without AGV base)
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=false

# 7. Object detector + pick
python3 scripts/hardware/dark_object_detector.py
python3 scripts/hardware/hw_pick.py
```

### Simulation

```bash
# Terminal 1: Gazebo + arm + camera
ros2 launch aagv_description gazebo.launch.py

# Terminal 2: Perception
ros2 launch aagv_perception perception.launch.py

# Terminal 3: MoveIt
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true
```

## Author

Handaru Rizqi W. — Teknologi Rekayasa Otomasi, Vokasi ITS
