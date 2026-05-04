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

class MoveGroupClient(Node):
    def __init__(self, planning_group, target_marker_frame):
        super().__init__('move_to_marker_client')
        
        self.planning_group = planning_group
        self.target_marker_frame = target_marker_frame
        self.pose_link = "tcp_offset_right" if "right" in self.planning_group else "tcp_offset_left"
        self.global_frame = "base" # MoveIt의 기준 프레임 (필요시 "base_link"로 변경)

        # TF2 버퍼 및 리스너 생성 (비동기로 계속 TF 트리를 수집합니다)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info(f"⏳ Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info(f"✅ Connected to MoveGroup action server!")
        
    def get_marker_pose_in_base(self):
        """ TF 버퍼에서 target_marker_frame을 찾아 Pose로 반환 (최대 5초 대기) """
        self.get_logger().info(f"🔍 '{self.target_marker_frame}' 타겟의 '{self.global_frame}' 기준 위치 수색 중...")
        
        for i in range(10): # 0.5초씩 10번 (총 5초) 기다리기
            try:
                # can_transform 체크 없이 바로 lookup_transform을 시도 (최신 정보 획득 시도)
                trans = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.target_marker_frame,
                    rclpy.time.Time()
                )
                
                target_pose = Pose()
                target_pose.position.x = trans.transform.translation.x
                target_pose.position.y = trans.transform.translation.y
                target_pose.position.z = trans.transform.translation.z
                target_pose.orientation.x = trans.transform.rotation.x
                target_pose.orientation.y = trans.transform.rotation.y
                target_pose.orientation.z = trans.transform.rotation.z
                target_pose.orientation.w = trans.transform.rotation.w
                
                self.get_logger().info(f"✅ TF 획득 성공: X({target_pose.position.x:.3f}), Y({target_pose.position.y:.3f}), Z({target_pose.position.z:.3f})")
                return target_pose
            except Exception as e:
                self.get_logger().warn(f"[{i+1}/10] 타겟 TF 대기 중... ({e})")
                time.sleep(0.5) # 실제 벽시계 시간으로 0.5초를 쉬어줍니다.
                
        self.get_logger().error(f"❌ TF 획득 실패! 프레임 '{self.target_marker_frame}' 이(가) 현재 TF상에 존재하지 않습니다.")
        return None

    def send_goal(self):
        target_pose = self.get_marker_pose_in_base()
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
        req.num_planning_attempts = 20
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.start_state.is_diff = True

        constraints = Constraints()
        
        # 1. 위치(Position) 제약
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.global_frame  
        pos_constraint.link_name = self.pose_link
        
        vol = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]  # 1센치 허용 오차
        vol.primitives.append(sphere)
        
        primitive_pose = Pose()
        primitive_pose.position.x = target_pose.position.x
        primitive_pose.position.y = target_pose.position.y
        primitive_pose.position.z = target_pose.position.z
        primitive_pose.orientation.w = 1.0 
        vol.primitive_poses.append(primitive_pose)
        
        pos_constraint.constraint_region = vol
        pos_constraint.weight = 1.0

        # 2. 회전(Orientation) 제약 (현재 주석 처리 - 마커의 회전 각도 기구학적 한계 방지)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.global_frame 
        ori_constraint.link_name = self.pose_link
        ori_constraint.orientation = target_pose.orientation 
        ori_constraint.absolute_x_axis_tolerance = 0.001
        ori_constraint.absolute_y_axis_tolerance = 0.001
        ori_constraint.absolute_z_axis_tolerance = 0.001
        ori_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        req.goal_constraints.append(constraints)

        goal_msg.request = req
        goal_msg.planning_options.plan_only = True 

        self.get_logger().info(f"🚀 '{self.planning_group}' 그룹의 TCP({self.pose_link}) 위치를 마커 절대좌표로 이동 계획 (Plan Only)...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Future 대기 동안 ROS가 멈추지 않도록 직접 대기 로직 사용
        while not send_goal_future.done():
            time.sleep(0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 계획 요청 거부됨!')
            return

        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.1)
        
        result_msg = get_result_future.result().result
        error_code = result_msg.error_code.val
        
        if error_code == 1:
            self.get_logger().info("✅ 궤적 계획 성공! RViz에서 주황색(계획된) 궤적을 확인하세요.")
            self.get_logger().info("⏳ 5초 뒤에 실제 로봇 이동을 실행합니다...")
            for i in range(5, 0, -1):
                self.get_logger().info(f"이동 시작까지 {i}초...")
                time.sleep(1.0)
            self.execute_trajectory(result_msg.planned_trajectory)
        else:
            self.get_logger().error(f"❌ 궤적 계획 실패! 에러 코드: {error_code} (마커가 작업 반경 밖이거나 충돌 지점에 위치함)")

    def execute_trajectory(self, trajectory):
        self.get_logger().info("🚀 로봇 이동을 시작합니다 (Execute Only)...")
        from moveit_msgs.action import ExecuteTrajectory
        exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        while not exec_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('waiting for execute_trajectory action server...')
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = exec_client.send_goal_async(goal_msg)
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 이동 요청 거부됨!')
            return
            
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)
            
        result = result_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info("🏁 이동 완료.")
        else:
            self.get_logger().error(f"❌ 이동 중 에러 발생 (코드: {result.error_code.val})")


def main():
    rclpy.init()

    # 제어 그룹과 타겟 마커
    planning_group = "rby1_left_arm"         
    target_marker = "captured_marker_1"       

    action_client = MoveGroupClient(planning_group, target_marker)
    
    # 노드의 ROS 콜백 처리를 별도 스레드로 분리하여 통신 병목 현상 완화
    spin_thread = threading.Thread(target=rclpy.spin, args=(action_client,), daemon=True)
    spin_thread.start()

    action_client.send_goal()
    
    action_client.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
