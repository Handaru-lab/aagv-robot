#!/usr/bin/env python3
"""
hw_pick_v2.py — Pick dengan MoveIt Pose goal (position + orientation).
Gripper selalu mengarah ke bawah (top-down approach).

Alur:
  1. Detect objek → transform ke world
  2. Open gripper
  3. MoveIt Pose goal: approach (di atas objek, gripper down)
  4. MoveIt Pose goal: descend (ke objek, gripper down)
  5. Close gripper
  6. MoveIt Pose goal: lift (angkat, gripper down)

Prasyarat (5 terminal):
  1. open_manipulator_x.launch.py port_name:=/dev/ttyACM0
  2. rs_launch.py ...
  3. static_transform_publisher link5 → camera_link
  4. dark_object_detector.py
  5. open_manipulator_x_moveit.launch.py use_sim:=false
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MotionPlanRequest, RobotState,
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


# Quaternion for EE pointing DOWN in world frame.
# Init pose: EE x-axis = world +x (forward), z-axis = world +z (up).
# Gripper-down = rotate EE so its x-axis points world -z.
# This is Ry(-90deg): quat (x=0, y=-sin(45)=-0.7071, z=0, w=cos(45)=0.7071)
Q_DOWN = Quaternion(x=0.0, y=-0.7071068, z=0.0, w=0.7071068)

# Tolerance for orientation constraint (radians)
# 4-DOF can't achieve arbitrary orientation — give generous tolerance
ORI_TOL = 0.4  # ~23 degrees tolerance on each axis


class HWPickV2(Node):
    def __init__(self):
        super().__init__('hw_pick_v2')

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # Action clients
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.grip_client = ActionClient(
            self, GripperCommandAction, '/gripper_controller/gripper_cmd')

        # State
        self.joint_state = None
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.cb_js, 10)

        self.object_world = None
        self.detections = []
        self.sub_obj = self.create_subscription(
            PointStamped, '/object/point', self.cb_object, 10)

        self.get_logger().info('HWPickV2 started. Waiting for detection...')

    def cb_js(self, msg):
        self.joint_state = msg

    def cb_object(self, msg):
        try:
            msg_copy = PointStamped()
            msg_copy.header.frame_id = msg.header.frame_id
            msg_copy.header.stamp.sec = 0
            msg_copy.header.stamp.nanosec = 0
            msg_copy.point = msg.point
            pt_world = self.tf_buf.transform(msg_copy, 'world', timeout=Duration(seconds=1))
            self.object_world = pt_world
            self.detections.append((pt_world.point.x, pt_world.point.y, pt_world.point.z))
            # Keep last 10
            if len(self.detections) > 10:
                self.detections.pop(0)
        except Exception as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=3.0)

    def wait_for_ready(self, timeout=15.0):
        self.get_logger().info('Waiting for MoveGroup...')
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('MoveGroup not available'); return False
        self.get_logger().info('Waiting for Gripper...')
        if not self.grip_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Gripper not available'); return False

        t0 = time.time()
        while self.joint_state is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('No joint state'); return False

        self.get_logger().info('Waiting for detection (up to 15s)...')
        t0 = time.time()
        while self.object_world is None and (time.time() - t0) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.object_world is None:
            self.get_logger().error('No detection'); return False

        p = self.object_world.point
        self.get_logger().info(f'Object in world: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        return True

    def get_median_detection(self):
        """Get median of recent detections for stability."""
        if not self.detections:
            p = self.object_world.point
            return p.x, p.y, p.z
        arr = np.array(self.detections)
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1])), float(np.median(arr[:, 2]))

    def move_pose(self, x, y, z, quat, label='move', pos_tol=0.015, attempts=10):
        """Move end_effector_link to (x,y,z) with orientation quat in world frame."""
        self.get_logger().info(
            f'[{label}] Pose goal: pos=({x:.3f},{y:.3f},{z:.3f})')

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'

        # Start state
        start = RobotState()
        start.joint_state = self.joint_state
        req.start_state = start

        # Position constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'world'
        pos_con.link_name = 'end_effector_link'
        pos_con.weight = 1.0
        region = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [pos_tol]
        region.primitives.append(prim)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region

        # Orientation constraint
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = 'world'
        ori_con.link_name = 'end_effector_link'
        ori_con.orientation = quat
        ori_con.absolute_x_axis_tolerance = ORI_TOL
        ori_con.absolute_y_axis_tolerance = ORI_TOL
        ori_con.absolute_z_axis_tolerance = ORI_TOL
        ori_con.weight = 0.8

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        req.goal_constraints.append(constraints)

        req.num_planning_attempts = attempts
        req.allowed_planning_time = 8.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Send
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'[{label}] Goal rejected')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info(f'[{label}] Success! (error_code=1)')
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)
            return True
        else:
            self.get_logger().error(f'[{label}] Failed (error_code={ec})')
            # Fallback: try without orientation constraint
            self.get_logger().info(f'[{label}] Retrying position-only...')
            return self.move_pos_only(x, y, z, label + '_fallback')

    def move_pos_only(self, x, y, z, label='move'):
        """Fallback: position-only (no orientation constraint)."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        start = RobotState()
        start.joint_state = self.joint_state
        req.start_state = start

        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'world'
        pos_con.link_name = 'end_effector_link'
        pos_con.weight = 1.0
        region = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.02]
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
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info(f'[{label}] Success!')
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)
            return True
        self.get_logger().error(f'[{label}] Failed (error_code={ec})')
        return False

    def gripper(self, position, label='gripper'):
        self.get_logger().info(f'[{label}] position={position:.3f}')
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

    def run_pick(self):
        ox, oy, oz = self.get_median_detection()

        # Safety
        r = math.sqrt(ox**2 + oy**2)
        if r > 0.33 or r < 0.08:
            self.get_logger().error(f'Object r={r:.3f}m out of reach. Abort.')
            return False
        if oz < -0.05 or oz > 0.25:
            self.get_logger().error(f'Object z={oz:.3f}m unusual. Abort.')
            return False

        self.get_logger().info(
            f'=== PICK V2 === target=({ox:.3f}, {oy:.3f}, {oz:.3f}), r={r:.3f}m')

        # 1. Open gripper
        self.gripper(0.019, 'open')

        # 2. Move to "home" pose first (safe intermediate)
        #    home: j1=0, j2=-1.0, j3=0.7, j4=0.3 — arm menunduk ke depan
        #    This ensures we start from a known good config
        self.get_logger().info('[home] Going to home pose first...')
        if not self.move_pose(0.236, 0.0, 0.051, Q_DOWN, 'home', pos_tol=0.03):
            self.get_logger().warn('[home] Could not reach home via Pose, trying named state')

        # 3. Approach: above object, gripper pointing down
        approach_z = oz + 0.07
        if not self.move_pose(ox, oy, approach_z, Q_DOWN, 'approach'):
            return False

        # 4. Descend: to grasp height, gripper pointing down
        grasp_z = oz + 0.005  # almost at object surface
        if not self.move_pose(ox, oy, grasp_z, Q_DOWN, 'descend', pos_tol=0.02):
            return False

        # 5. Close gripper
        self.gripper(-0.01, 'close')
        time.sleep(0.5)  # let gripper settle

        # 6. Lift
        lift_z = oz + 0.10
        if not self.move_pose(ox, oy, lift_z, Q_DOWN, 'lift'):
            return False

        self.get_logger().info('=== PICK V2 COMPLETE ===')
        return True


def main():
    rclpy.init()
    node = HWPickV2()

    if not node.wait_for_ready():
        node.get_logger().error('Not ready.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Collect detections
    node.get_logger().info('Collecting detections for 3 seconds...')
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    ox, oy, oz = node.get_median_detection()
    node.get_logger().info(
        f'Median target: ({ox:.3f}, {oy:.3f}, {oz:.3f}) from {len(node.detections)} samples')
    node.get_logger().info('Starting pick in 3 seconds... HOLD ARM READY.')
    time.sleep(3.0)

    success = node.run_pick()
    if success:
        node.get_logger().info('Pick succeeded!')
    else:
        node.get_logger().error('Pick failed.')

    node.get_logger().info('Ctrl+C to exit.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
