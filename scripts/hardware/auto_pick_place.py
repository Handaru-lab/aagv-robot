#!/usr/bin/env python3
"""
auto_pick_place.py — Autonomous scan → detect → pick → place.

Scan start pose: j2=-75°, j3=+75°, j4=0° (camera horizontal, arm bent).
Scan down: j4 increases by +10° per level (positive = wrist tilts down in MoveIt).
8 compass directions (7 valid) × up to 19 levels (0° to -90° camera pitch).
Early exit on first reachable detection.

Planning frame: link1.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, JointConstraint,
    BoundingVolume, RobotState,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand as GripperCommandAction
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import numpy as np
import math
import sys
import time


PLANNING_FRAME = 'link1'

# ============================================================
# Scan configuration
# ============================================================
COMPASS = [
    ('N',   0.000),
    ('NE',  0.785),
    ('E',   1.571),
    ('SE',  2.356),
    ('SW', -2.356),
    ('W',  -1.571),
    ('NW', -0.785),
]

# Start pose: j2=-75°, j3=+75°, j4=0°  → camera pitch = 0° (horizontal)
J2_SCAN = math.radians(-75)  # -1.309 rad
J3_SCAN = math.radians(75)   #  1.309 rad

# j4 starts at 0, increases by +10° per level (positive = menunduk in MoveIt)
DEG10 = math.radians(10)
MAX_LEVELS = 10  # 0° to +90° (10 levels × 10°)
# j4 limit: -1.885 rad (-108°)

SCAN_ORDER = []
for level in range(MAX_LEVELS):
    j4 = level * DEG10  # 0, +10°, +20°, ..., +90°
    if j4 > 2.147:
        break
    cam_pitch_deg = math.degrees(J2_SCAN + J3_SCAN + j4)
    for dir_name, j1 in COMPASS:
        SCAN_ORDER.append({
            'name': f'{dir_name}_L{level}(cam{cam_pitch_deg:.0f}deg)',
            'joints': [j1, J2_SCAN, J3_SCAN, j4],
        })

DEFAULT_PLACE = (0.15, 0.15, 0.05)
HOME_JOINTS = [0.0, math.radians(-75), math.radians(75), 0.0]  # scan start = home


class AutoPickPlace(Node):
    def __init__(self):
        super().__init__('auto_pick_place')

        self.declare_parameter('place_x', DEFAULT_PLACE[0])
        self.declare_parameter('place_y', DEFAULT_PLACE[1])
        self.declare_parameter('place_z', DEFAULT_PLACE[2])
        self.declare_parameter('scan_dwell', 1.5)
        self.declare_parameter('detect_min_count', 3)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.grip_client = ActionClient(
            self, GripperCommandAction, '/gripper_controller/gripper_cmd')

        self.joint_state = None
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.cb_js, 10)

        self.det_buffer = []
        self.sub_obj = self.create_subscription(
            PointStamped, '/object/point', self.cb_object, 10)

        n_poses = len(SCAN_ORDER)
        n_levels = MAX_LEVELS
        self.get_logger().info(
            f'AutoPickPlace: {n_poses} scan poses '
            f'({len(COMPASS)} dirs × {n_levels} levels)')
        self.get_logger().info(
            f'  Start pose: j2={math.degrees(J2_SCAN):.0f}° j3={math.degrees(J3_SCAN):.0f}° j4=0°')
        self.get_logger().info(
            f'  Scan: j4 tilts +10° per level (positive=down) ')

    def cb_js(self, msg):
        self.joint_state = msg

    def cb_object(self, msg):
        try:
            msg_copy = PointStamped()
            msg_copy.header.frame_id = msg.header.frame_id
            msg_copy.header.stamp.sec = 0
            msg_copy.header.stamp.nanosec = 0
            msg_copy.point = msg.point
            pt = self.tf_buf.transform(msg_copy, PLANNING_FRAME,
                                       timeout=Duration(seconds=0, nanoseconds=500_000_000))
            self.det_buffer.append((pt.point.x, pt.point.y, pt.point.z))
        except Exception:
            pass

    def wait_for_ready(self, timeout=15.0):
        self.get_logger().info('Waiting for action servers...')
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('MoveGroup not available')
            return False
        if not self.grip_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Gripper not available')
            return False
        t0 = time.time()
        while self.joint_state is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('No joint state')
            return False
        self.get_logger().info('Ready.')
        return True

    def move_joints(self, joint_values, label='move'):
        self.get_logger().info(
            f'[{label}] j=[{", ".join(f"{math.degrees(v):.1f}°" for v in joint_values)}]')

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        start = RobotState()
        start.joint_state = self.joint_state
        req.start_state = start

        constraints = Constraints()
        for name, val in zip(['joint1', 'joint2', 'joint3', 'joint4'], joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'[{label}] Rejected')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info(f'[{label}] OK')
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)
            return True
        self.get_logger().error(f'[{label}] Failed (ec={ec})')
        return False

    def move_cartesian(self, x, y, z, label='move', pos_tol=0.015):
        self.get_logger().info(f'[{label}] ({x:.3f}, {y:.3f}, {z:.3f}) in {PLANNING_FRAME}')

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        start = RobotState()
        start.joint_state = self.joint_state
        req.start_state = start

        pos_con = PositionConstraint()
        pos_con.header.frame_id = PLANNING_FRAME
        pos_con.link_name = 'end_effector_link'
        pos_con.weight = 1.0
        region = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [pos_tol]
        region.primitives.append(prim)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        region.primitive_poses.append(p)
        pos_con.constraint_region = region

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        req.goal_constraints.append(constraints)

        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'[{label}] Rejected')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info(f'[{label}] OK')
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.2)
            return True
        self.get_logger().error(f'[{label}] Failed (ec={ec})')
        return False

    def gripper(self, position, label='gripper'):
        self.get_logger().info(f'[{label}] pos={position:.3f}')
        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        future = self.grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if not gh or not gh.accepted:
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        self.get_logger().info(f'[{label}] Done')
        time.sleep(0.3)
        return True

    def scan_for_object(self):
        dwell = self.get_parameter('scan_dwell').value
        min_count = self.get_parameter('detect_min_count').value

        self.get_logger().info(
            f'=== SCAN === {len(SCAN_ORDER)} poses, dwell={dwell}s')

        for i, pose in enumerate(SCAN_ORDER):
            name = pose['name']
            joints = pose['joints']

            self.det_buffer.clear()

            if not self.move_joints(joints, f'scan_{name}'):
                self.get_logger().warn(f'[scan] {name} unreachable, skip')
                continue

            t0 = time.time()
            while time.time() - t0 < dwell:
                rclpy.spin_once(self, timeout_sec=0.05)

            if len(self.det_buffer) < min_count:
                self.get_logger().info(
                    f'[scan {i+1}/{len(SCAN_ORDER)}] {name}: '
                    f'{len(self.det_buffer)} det (need {min_count})')
                continue

            arr = np.array(self.det_buffer)
            mx = float(np.median(arr[:, 0]))
            my = float(np.median(arr[:, 1]))
            mz = float(np.median(arr[:, 2]))
            r = math.sqrt(mx**2 + my**2)

            self.get_logger().info(
                f'[scan {i+1}/{len(SCAN_ORDER)}] {name}: '
                f'FOUND ({mx:.3f}, {my:.3f}, {mz:.3f}) r={r:.3f}m '
                f'[{len(self.det_buffer)} samples]')

            if 0.08 <= r <= 0.30 and -0.25 <= mz <= 0.15:
                self.get_logger().info('[scan] IN REACH → stop scan')
                return (mx, my, mz)
            else:
                self.get_logger().warn(
                    f'[scan] Out of reach (r={r:.3f} z={mz:.3f}), continue')

        self.get_logger().warn('=== SCAN DONE === No reachable object.')
        return None

    def pick(self, ox, oy, oz):
        self.get_logger().info(f'=== PICK === ({ox:.3f}, {oy:.3f}, {oz:.3f})')
        self.gripper(0.019, 'open')

        if not self.move_cartesian(ox, oy, oz + 0.04, 'approach'):
            return False
        if not self.move_cartesian(ox, oy, oz + 0.005, 'descend', pos_tol=0.02):
            return False

        self.gripper(-0.01, 'close')
        time.sleep(0.5)

        if not self.move_cartesian(ox, oy, oz + 0.10, 'lift'):
            return False

        self.get_logger().info('=== PICK DONE ===')
        return True

    def place(self):
        px = self.get_parameter('place_x').value
        py = self.get_parameter('place_y').value
        pz = self.get_parameter('place_z').value

        self.get_logger().info(f'=== PLACE === ({px:.3f}, {py:.3f}, {pz:.3f})')

        if not self.move_cartesian(px, py, pz + 0.06, 'place_approach'):
            return False
        if not self.move_cartesian(px, py, pz, 'place_lower', pos_tol=0.02):
            return False

        self.gripper(0.019, 'release')
        time.sleep(0.3)
        self.move_cartesian(px, py, pz + 0.08, 'place_retreat')

        self.get_logger().info('=== PLACE DONE ===')
        return True

    def go_home(self):
        return self.move_joints(HOME_JOINTS, 'home')

    def run(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  AUTO PICK-PLACE — Single Cycle')
        self.get_logger().info(f'  Start: j2=-75° j3=+75° j4=0° (cam horizontal)')
        self.get_logger().info(f'  Scan: j4 tilts +10°/level, 7 dirs per level')
        self.get_logger().info('=' * 60)

        target = self.scan_for_object()
        if target is None:
            self.get_logger().error('No object. Going home.')
            self.go_home()
            return False

        ox, oy, oz = target
        self.get_logger().info(f'Target: ({ox:.3f}, {oy:.3f}, {oz:.3f}). Pick in 2s...')
        time.sleep(2.0)

        if not self.pick(ox, oy, oz):
            self.get_logger().error('Pick failed. Going home.')
            self.go_home()
            return False

        if not self.place():
            self.get_logger().error('Place failed. Going home.')
            self.go_home()
            return False

        self.go_home()
        self.get_logger().info('=' * 60)
        self.get_logger().info('  CYCLE COMPLETE!')
        self.get_logger().info('=' * 60)
        return True


def main():
    rclpy.init()
    node = AutoPickPlace()
    if not node.wait_for_ready():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info('Starting in 3 seconds... HOLD ARM.')
    time.sleep(3.0)

    success = node.run()
    node.get_logger().info('Ctrl+C to exit.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
