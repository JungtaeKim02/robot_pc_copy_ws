# 단일 grasp 후보가 파이프라인에서 어떻게 처리됐는지를 기록.
# 각 노드가 처리한 결과를 누적하여 최종적으로 RequestSummary 안에 들어간다.

# ----- 식별 -----
uint32 candidate_index           # 0-based, 입력 PoseArray 의 인덱스
uint32 variant_index              # prefilter 에서 yaw flip 변형 인덱스 (0=원본, 1=flip)
                                  # action_server 단계까지 살아남은 후보는
                                  # variant_index 가 0/1 중 하나로 고정됨

# ----- pose -----
geometry_msgs/Pose source_pose    # 원본 입력 (input_frame 기준)
geometry_msgs/Pose pregrasp_world # standoff 적용 후 (planning frame)
geometry_msgs/Pose realgrasp_world # standoff 미적용 (planning frame)

# ----- 어디서 멈췄나 -----
# 가능한 값 (시간순):
#   "input"                    : 입력 단계만 통과 (manager 가 publish 직후)
#   "manager_unreachable"      : reachability map 에서 reject
#   "rigid_block_blocked"      : prefilter rigid block check 실패
#   "camera_collision_blocked" : prefilter 카메라 charge check 실패
#   "approach_path_blocked"    : action_server [개선 1]
#   "realgrasp_ik_failed"      : action_server [개선 2-②]
#   "pregrasp_ik_failed"       : action_server [개선 2-③]
#   "ik_imprecise"             : pos/orient 오차가 임계 초과
#   "step1_plan_failed"        : MoveIt plan() 실패
#   "step1_exec_failed"
#   "step2_low_coverage"       : Cartesian coverage 부족
#   "step2_misaligned"         : 이동 방향 alignment 부족
#   "step2_exec_failed"
#   "winner_executed"          : 최종 성공
#   "not_attempted"            : 상위 후보가 성공하여 시도되지 않음
string final_stage
string failure_reason             # 사람이 읽을 수 있는 상세 사유

# ----- 단계별 정량 지표 -----
# 해당 단계까지 도달한 경우만 채워지고, 아니면 NaN.
float64 manipulability
float64 ns_cost
float64 step2_coverage_percent
float64 step2_trim_distance_m
float64 step2_direction_alignment

# [개선 2 검증용] q_realgrasp 와 q_pregrasp 사이의 joint-space 거리
# 작을수록 의도대로 동작 — Step2 가 안정적임.
float64 q_realgrasp_pregrasp_distance_rad

# IK 해 (winner 가 됐다면 채움. 그 외 NaN/empty)
float64[] ik_solution_pregrasp
float64[] ik_solution_realgrasp

# ----- 타이밍 -----
float64 elapsed_ms                # 이 후보를 처리하는 데 걸린 시간 (해당 노드 내)
