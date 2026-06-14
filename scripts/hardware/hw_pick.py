#!/usr/bin/env python3
"""
hw_pick.py — Orchestrator pick objek gelap pada hardware nyata.
Alur: detect → tf2 transform ke world → MoveIt approach → descend → grip → lift.

Prasyarat yang harus jalan:
  1. open_manipulator_x.launch.py port_name:=/dev/ttyACM0  (arm controller)
  2. rs_launch.py ...  (kamera RealSense)
  3. static_transform_publisher link5 → camera_link
  4. dark_object_detector.py  (publish /object/point)

Jalankan:
  python3 ~/hw_pick.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MotionPlanRequest, PlanningOptions,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registers PointStamped transform
import numpy as np
import sys
import time


class HWPick(Node):
    def __init__(self):
        super().__init__('hw_pick')

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # MoveGroup action client
        self.move_client = ActionClient(self, MoveGroup, 'move_action')

        # Gripper action client
        self.grip_client = ActionClient(
            self, GripperCommandAction, '/gripper_controller/gripper_cmd')

        # Joint state
        self.joint_state = None
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.cb_js, 10)

        # Object detection
        self.object_world = None
        self.sub_obj = self.create_subscription(
            PointStamped, '/object/point', self.cb_object, 10)

        self.get_logger().info('HWPick node started. Waiting for detection...')

    def cb_js(self, msg):
        self.joint_state = msg

    def cb_object(self, msg):
        """Transform detected object to world frame."""
        try:
            # Use latest available TF (Time=0) to avoid extrapolation errors
            msg_copy = PointStamped()
            msg_copy.header.frame_id = msg.header.frame_id
            msg_copy.header.stamp.sec = 0
            msg_copy.header.stamp.nanosec = 0
            msg_copy.point = msg.point
            pt_world = self.tf_buf.transform(msg_copy, 'world', timeout=Duration(seconds=1))
            self.object_world = pt_world
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=2.0)

    def wait_for_ready(self, timeout=15.0):
        """Wait for action servers + joint state + detection."""
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('MoveGroup not available')
            return False
        self.get_logger().info('Waiting for Gripper action server...')
        if not self.grip_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Gripper not available')
            return False

        # Wait for joint state
        t0 = time.time()
        while self.joint_state is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('No joint state received')
            return False

        # Wait for detection
        self.get_logger().info('Waiting for object detection (up to 10s)...')
        t0 = time.time()
        while self.object_world is None and (time.time() - t0) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.object_world is None:
            self.get_logger().error('No object detected')
            return False

        p = self.object_world.point
        self.get_logger().info(
            f'Object in world: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')
        return True

    def move_to(self, x, y, z, label='move'):
        """Move end_effector_link to (x,y,z) in world frame using MoveGroup.
        Orientation: gripper pointing down (-Z in world)."""
        self.get_logger().info(f'[{label}] Moving to ({x:.3f}, {y:.3f}, {z:.3f})...')

        goal = MoveGroup.Goal()
        req = goal.request

        # Planning group
        req.group_name = 'arm'

        # Start state = current
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
        prim.dimensions = [0.02]  # 2cm tolerance
        region.primitives.append(prim)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region

        constraints = Constraints()
        constraints.position_constraints.append(pos_con)
        req.goal_constraints.append(constraints)

        # Planning options
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Send goal
        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'[{label}] Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        error_code = result.result.error_code.val
        if error_code == 1:
            self.get_logger().info(f'[{label}] Success!')
            # Update joint state
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.2)
            return True
        else:
            self.get_logger().error(f'[{label}] Failed, error_code={error_code}')
            return False

    def gripper(self, position, label='gripper'):
        """Command gripper. position: -0.01 (closed) to 0.019 (open)."""
        self.get_logger().info(f'[{label}] Gripper position={position:.3f}')
        goal = GripperCommandAction.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0

        future = self.grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'[{label}] Gripper goal rejected')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        self.get_logger().info(f'[{label}] Gripper done')
        time.sleep(0.5)
        return True

    def run_pick(self):
        p = self.object_world.point
        obj_x, obj_y, obj_z = p.x, p.y, p.z

        # Safety check: is object within reach?
        r = np.sqrt(obj_x**2 + obj_y**2)
        if r > 0.35 or r < 0.10:
            self.get_logger().error(
                f'Object at r={r:.3f}m — out of safe reach (0.10-0.35m). Abort.')
            return False
        if obj_z < -0.05 or obj_z > 0.20:
            self.get_logger().error(
                f'Object z={obj_z:.3f}m — unusual height. Abort.')
            return False

        self.get_logger().info(
            f'=== PICK SEQUENCE === target=({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})')

        # Step 1: Open gripper
        self.gripper(0.019, 'open_gripper')

        # Step 2: Approach — above object
        approach_z = obj_z + 0.06  # 6cm above
        if not self.move_to(obj_x, obj_y, approach_z, 'approach'):
            return False

        # Step 3: Descend to object
        # Penghapus tidur ~3cm tinggi. Gripper finger ~2.5cm.
        # Grip di tengah objek = obj_z + 0.005 (hampir rata permukaan)
        grasp_z = obj_z + 0.005
        if not self.move_to(obj_x, obj_y, grasp_z, 'descend'):
            return False

        # Step 4: Close gripper
        self.gripper(-0.01, 'close_gripper')

        # Step 5: Lift
        lift_z = obj_z + 0.12  # 12cm above original
        if not self.move_to(obj_x, obj_y, lift_z, 'lift'):\
            return False

        self.get_logger().info('=== PICK COMPLETE ===')
        return True


def main():
    rclpy.init()
    node = HWPick()

    if not node.wait_for_ready():
        node.get_logger().error('Not ready. Check all launches are running.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Collect a few more detections for stability
    node.get_logger().info('Collecting detections for 2 seconds...')
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    p = node.object_world.point
    node.get_logger().info(
        f'Final target in world: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
    node.get_logger().info('Starting pick in 3 seconds... HOLD THE ARM READY TO CATCH.')
    time.sleep(3.0)

    success = node.run_pick()
    if success:
        node.get_logger().info('Pick succeeded! Object should be lifted.')
    else:
        node.get_logger().error('Pick failed. Check logs above.')

    node.get_logger().info('Node will stay alive. Ctrl+C to exit.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
