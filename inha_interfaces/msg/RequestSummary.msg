# 한 번의 manipulation 요청 (= /manipulation/moveit/enable 한 번 호출) 에
# 대한 전체 결과 요약. 각 노드가 자신의 단계 결과를 publish 하지만,
# action_server 가 최종적으로 모든 단계를 합쳐 이 메시지를 발행한다.

std_msgs/Header header

# ----- 식별 -----
string request_id                 # manager 에서 생성된 UUID
string arm_id                     # "left" / "right"
string mode                       # "grasp" / "place"
string pose_source                # "graspgen" / "cutlery" / "anyplace"

# ----- 최종 결과 -----
bool overall_success
int32 winning_candidate_index     # -1 if failed
int32 winning_variant_index       # -1 if failed
uint32 winning_batch_index
uint32 winning_rank

string final_status_message       # 사람이 읽는 결과 설명

# ----- 단계별 통과 카운트 (한 눈에 어디서 빠지는지) -----
uint32 n_input
uint32 n_after_manager_reach      # reachability map 통과
uint32 n_after_prefilter          # rigid_block + camera 통과 (variants 합산 후)
uint32 n_after_approach_filter    # 개선 1 통과
uint32 n_realgrasp_ik_ok          # 개선 2 - ② 통과
uint32 n_pregrasp_ik_ok           # 개선 2 - ③ 통과
uint32 n_step1_plan_ok
uint32 n_step2_cartesian_ok

# ----- 모든 후보의 상세 (실패 후보 포함) -----
inha_interfaces/CandidateResult[] candidates

# ----- 타이밍 -----
float64 total_elapsed_ms
