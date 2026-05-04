#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # PoseStamped 변환용


class CutleryPlanClient(Node):
    def __init__(self):
        super().__init__('cutlery_plan_client')

        self.planning_group = "rby1_right_arm"
        self.pose_link = "ee_right"
        self.global_frame = "base"

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # goal_pose 수신
        self.latest_goal_pose = None
        self.create_subscription(
            PoseStamped, '/cutlery/goal_pose',
            self._goal_pose_cb, 10)

        # MoveGroup action client
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server!")

    def _goal_pose_cb(self, msg: PoseStamped):
        self.latest_goal_pose = msg

    def get_goal_pose_in_base(self):
        """cutlery/goal_pose를 base 프레임으로 변환"""
        self.get_logger().info("Waiting for /cutlery/goal_pose...")

        for i in range(20):  # 10초 대기
            if self.latest_goal_pose is not None:
                break
            time.sleep(0.5)

        if self.latest_goal_pose is None:
            self.get_logger().error("/cutlery/goal_pose not received!")
            return None

        pose_msg = self.latest_goal_pose
        src_frame = pose_msg.header.frame_id
        self.get_logger().info(
            f"Received goal_pose [{src_frame}]: "
            f"({pose_msg.pose.position.x:.4f}, "
            f"{pose_msg.pose.position.y:.4f}, "
            f"{pose_msg.pose.position.z:.4f})")

        # 이미 base 프레임이면 그대로 사용
        if src_frame == self.global_frame:
            return pose_msg.pose

        # base 프레임으로 변환
        for i in range(10):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame, src_frame,
                    rclpy.time.Time())
                transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_msg, transform)
                self.get_logger().info(
                    f"Transformed to [{self.global_frame}]: "
                    f"({transformed.pose.position.x:.4f}, "
                    f"{transformed.pose.position.y:.4f}, "
                    f"{transformed.pose.position.z:.4f})")
                return transformed.pose
            except Exception as e:
                self.get_logger().warn(f"[{i+1}/10] TF waiting... ({e})")
                time.sleep(0.5)

        self.get_logger().error(f"TF {src_frame} -> {self.global_frame} failed!")
        return None

    def send_goal(self):
        target_pose = self.get_goal_pose_in_base()
        if target_pose is None:
            return

        goal_msg = MoveGroup.Goal()

        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = self.global_frame
        req.workspace_parameters.min_corner.x = -3.0
        req.workspace_parameters.min_corner.y = -3.0
        req.workspace_parameters.min_corner.z = -3.0
        req.workspace_parameters.max_corner.x = 3.0
        req.workspace_parameters.max_corner.y = 3.0
        req.workspace_parameters.max_corner.z = 3.0

        req.group_name = self.planning_group
        req.num_planning_attempts = 50
        req.allowed_planning_time = 30.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.start_state.is_diff = True

        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.global_frame
        pos_constraint.link_name = self.pose_link

        vol = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        vol.primitives.append(sphere)

        primitive_pose = Pose()
        primitive_pose.position = target_pose.position
        primitive_pose.orientation.w = 1.0
        vol.primitive_poses.append(primitive_pose)

        pos_constraint.constraint_region = vol
        pos_constraint.weight = 1.0

        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.global_frame
        ori_constraint.link_name = self.pose_link
        ori_constraint.orientation = target_pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.7
        ori_constraint.absolute_y_axis_tolerance = 0.7
        ori_constraint.absolute_z_axis_tolerance = 0.7
        ori_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        #constraints.orientation_constraints.append(ori_constraint)
        req.goal_constraints.append(constraints)

        goal_msg.request = req
        goal_msg.planning_options.plan_only = True  # plan만, 실행 안 함

        self.get_logger().info(f"Planning {self.planning_group} -> cutlery goal_pose (Plan Only)...")

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Plan request rejected!")
            return

        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.1)

        result_msg = get_result_future.result().result
        error_code = result_msg.error_code.val

        if error_code == 1:
            self.get_logger().info("Plan SUCCESS! Check RViz for the planned trajectory.")
        else:
            self.get_logger().error(f"Plan FAILED! error_code: {error_code}")


def main():
    rclpy.init()
    client = CutleryPlanClient()

    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    client.send_goal()

    client.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
