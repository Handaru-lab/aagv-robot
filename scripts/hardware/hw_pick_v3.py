#!/usr/bin/env python3
"""
hw_pick_v3.py — Pick orchestrator compatible with AAGV MoveIt config.
Transforms object to 'link1' frame (arm base) to avoid AGV base offset issue.
Planning goals sent in 'link1' frame — MoveIt resolves via TF.

Prasyarat (6 terminal):
  1. open_manipulator_x.launch.py port_name:=/dev/ttyACM0
  2. aagv_moveit_config move_group.launch.py
  3. rs_launch.py ...
  4. static_transform_publisher link5 → camera_link
  5. unified_detector_ros.py --ros-args -p detection_mode:=dark -p publish_interval:=0.0
  6. python3 hw_pick_v3.py
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, BoundingVolume,
    MotionPlanRequest, RobotState,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand as GripperCommandAction
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import numpy as np
import sys
import time


PLANNING_FRAME = 'link1'  # arm base — avoids AGV base_link offset


class HWPickV3(Node):
    def __init__(self):
        super().__init__('hw_pick_v3')

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.grip_client = ActionClient(
            self, GripperCommandAction, '/gripper_controller/gripper_cmd')

        self.joint_state = None
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.cb_js, 10)

        self.object_link1 = None
        self.detections = []
        self.sub_obj = self.create_subscription(
            PointStamped, '/object/point', self.cb_object, 10)

        self.get_logger().info('HWPickV3 started (planning in link1 frame)')

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
                                       timeout=Duration(seconds=1))
            self.object_link1 = pt
            self.detections.append((pt.point.x, pt.point.y, pt.point.z))
            if len(self.detections) > 15:
                self.detections.pop(0)
        except Exception as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=3.0)

    def wait_for_ready(self, timeout=15.0):
        self.get_logger().info('Waiting for MoveGroup...')
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('MoveGroup not available')
            return False
        self.get_logger().info('Waiting for Gripper...')
        if not self.grip_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Gripper not available')
            return False

        t0 = time.time()
        while self.joint_state is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joint_state is None:
            self.get_logger().error('No joint state')
            return False

        self.get_logger().info('Waiting for detection (up to 15s)...')
        t0 = time.time()
        while self.object_link1 is None and (time.time() - t0) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.object_link1 is None:
            self.get_logger().error('No detection')
            return False

        p = self.object_link1.point
        self.get_logger().info(f'Object in {PLANNING_FRAME}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        return True

    def get_median_detection(self):
        if not self.detections:
            p = self.object_link1.point
            return p.x, p.y, p.z
        arr = np.array(self.detections)
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1])), float(np.median(arr[:, 2]))

    def move_to(self, x, y, z, label='move', pos_tol=0.015):
        self.get_logger().info(f'[{label}] Target in {PLANNING_FRAME}: ({x:.3f}, {y:.3f}, {z:.3f})')

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
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        region.primitive_poses.append(target_pose)
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
            self.get_logger().error(f'[{label}] Goal rejected')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result()
        ec = result.result.error_code.val
        if ec == 1:
            self.get_logger().info(f'[{label}] Success!')
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.2)
            return True
        else:
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

        r = np.sqrt(ox**2 + oy**2)
        self.get_logger().info(
            f'=== PICK V3 === target in {PLANNING_FRAME}: ({ox:.3f}, {oy:.3f}, {oz:.3f}), r={r:.3f}m')

        if r > 0.35 or r < 0.08:
            self.get_logger().error(f'r={r:.3f}m out of reach (0.08-0.35m). Abort.')
            return False
        if oz < -0.20 or oz > 0.20:
            self.get_logger().error(f'z={oz:.3f}m unusual. Abort.')
            return False

        # 1. Open gripper
        self.gripper(0.019, 'open')

        # 2. Approach — above object
        approach_z = oz + 0.06
        if not self.move_to(ox, oy, approach_z, 'approach'):
            return False

        # 3. Descend — to grasp height
        grasp_z = oz + 0.005
        if not self.move_to(ox, oy, grasp_z, 'descend', pos_tol=0.02):
            return False

        # 4. Close gripper
        self.gripper(-0.01, 'close')
        time.sleep(0.5)

        # 5. Lift
        lift_z = oz + 0.10
        if not self.move_to(ox, oy, lift_z, 'lift'):
            return False

        self.get_logger().info('=== PICK V3 COMPLETE ===')
        return True


def main():
    rclpy.init()
    node = HWPickV3()

    if not node.wait_for_ready():
        node.get_logger().error('Not ready.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info('Collecting detections for 3 seconds...')
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    ox, oy, oz = node.get_median_detection()
    node.get_logger().info(
        f'Median target in {PLANNING_FRAME}: ({ox:.3f}, {oy:.3f}, {oz:.3f}) '
        f'from {len(node.detections)} samples')
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
