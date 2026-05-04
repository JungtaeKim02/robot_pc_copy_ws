#!/usr/bin/env python3
"""
Phase 2 Step A: RBY1 yml sanity check (cuRobo v2 정확한 API).

검증 항목:
  (1) yml 로드 + Kinematics 생성
  (2) 기본 자세 FK -> ee_right 의 world pose 출력
  (3) ee_right 위치를 +X 5cm 옮긴 pose 에 IK 풀기
"""
import torch
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import JointState, Pose, GoalToolPose

YML_PATH = "/home/nvidia/curobo_ws/src/configs/rby1_inha.yml"


def main():
    # ==============================================================
    # (1) Kinematics 생성 (FK 만 쓸 때)
    # ==============================================================
    print("=== Loading RBY1 kinematics ===")
    kin_cfg = KinematicsCfg.from_robot_yaml_file(YML_PATH)
    robot = Kinematics(kin_cfg)

    print(f"  DOF (active joints): {robot.get_dof()}")
    print(f"  joint_names:         {robot.joint_names}")
    print(f"  tool_frames:         {robot.tool_frames}")

    # ==============================================================
    # (2) 기본 자세 FK
    # ==============================================================
    print("\n=== Default joint state -> FK ===")
    q_default = robot.default_joint_state.position           # tensor [DOF]
    print(f"  default position shape: {q_default.shape}")
    print(f"  default position:       {q_default}")

    # JointState 만들기 (batch dim 추가)
    q_input = q_default.unsqueeze(0)                          # [1, DOF]
    js = JointState.from_position(q_input, joint_names=robot.joint_names)

    state = robot.compute_kinematics(js)
    print(f"  state.tool_poses.position.shape:  {state.tool_poses.position.shape}")
    print(f"  state.tool_poses.tool_frames:     {state.tool_poses.tool_frames}")

    # tool_frames 의 첫 번째 (ee_right) pose 추출
    target_link = robot.tool_frames[0]
    ee_pose = state.tool_poses.get_link_pose(target_link)
    print(f"  EE link  : {target_link}")
    print(f"  EE position (world): {ee_pose.position}")
    print(f"  EE quaternion (wxyz): {ee_pose.quaternion}")

    # ==============================================================
    # (3) IK: 기본 자세에서 EE +X 5cm 옮긴 pose 에 IK
    # ==============================================================
    print("\n=== IK to target (EE +X 5cm) ===")

    # IK 솔버 생성 (collision check 없는 가벼운 형태)
    ik_cfg = InverseKinematicsCfg.create(
        robot=YML_PATH,
        num_seeds=32,
    )
    ik = InverseKinematics(ik_cfg)
    ik_target_link = ik.tool_frames[0]
    print(f"  IK tool_frame: {ik_target_link}")

    # target = ee_pose 의 위치를 +X 5cm 옮김
    target_position = ee_pose.position.clone()
    target_position[..., 0] += 0.05   # x +5cm
    target_quaternion = ee_pose.quaternion.clone()

    target_pose = Pose(
        position=target_position,
        quaternion=target_quaternion,
    )
    goal = GoalToolPose.from_poses({ik_target_link: target_pose}, num_goalset=1)

    result = ik.solve_pose(goal)

    if result.success.any():
        print(f"  ✓ IK succeeded")
        print(f"    position error: {result.position_error.item() * 1000:.3f} mm")
        if hasattr(result, 'rotation_error'):
            print(f"    rotation error: {result.rotation_error.item():.6f} rad")
        if hasattr(result, 'js_solution'):
            print(f"    solution joints: {result.js_solution.position}")
    else:
        print(f"  ✗ IK failed")
        if hasattr(result, 'status'):
            print(f"    status: {result.status}")


if __name__ == "__main__":
    main()
