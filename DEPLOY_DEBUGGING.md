# 로봇 PC 배포 가이드 — v2 디버깅 시스템

이 문서는 본 워크스페이스 (`robot_pc_copy_ws`) 에서 작성한 변경사항을
실제 로봇 PC 에 적용하기 위한 안내입니다.

---

## 1. 변경사항 요약

| 항목 | 본 워크스페이스 경로 | 로봇 PC 경로 |
|------|---------------------|--------------|
| **신규 메시지** | `inha_interfaces/msg/CandidateResult.msg` | `<ros2_ws>/src/inha_interfaces/msg/CandidateResult.msg` |
| **신규 메시지** | `inha_interfaces/msg/RequestSummary.msg` | `<ros2_ws>/src/inha_interfaces/msg/RequestSummary.msg` |
| **srv 수정** | `inha_interfaces/srv/ExecuteGraspSrv.srv.PATCH` (안내) | `<ros2_ws>/src/inha_interfaces/srv/ExecuteGraspSrv.srv` |
| **CMakeLists 수정** | (없음) | `<ros2_ws>/src/inha_interfaces/CMakeLists.txt` |
| **신규 cpp** | `inha_control_client/src/rby1_action_server_bio_ik_v2.cpp` | `<ros2_ws>/src/inha_control_client/src/rby1_action_server_bio_ik_v2.cpp` |
| **cpp 수정** | `inha_control_client/src/rby1_planner_client.cpp` | 동일 경로 |
| **cpp 수정** | `inha_control_client/src/grasp_prefilter_node.cpp` | 동일 경로 |
| **CMakeLists 수정** | `inha_control_client/CMakeLists.txt` | 동일 경로 |

---

## 2. 작업 순서

### Step 1 — `inha_interfaces` 패키지 업데이트

```bash
cd <ros2_ws>/src/inha_interfaces

# 1) 메시지 파일 두 개 추가
cp <copy_ws>/inha_interfaces/msg/CandidateResult.msg ./msg/
cp <copy_ws>/inha_interfaces/msg/RequestSummary.msg  ./msg/

# 2) ExecuteGraspSrv.srv 의 Request 부분에 한 줄 추가
#    (--- 위 영역에)
#       string request_id
#    상세는 .PATCH 파일 참조

# 3) CMakeLists.txt 수정
#    rosidl_generate_interfaces(${PROJECT_NAME} ...) 블록에
#    아래 두 줄 추가:
#       "msg/CandidateResult.msg"
#       "msg/RequestSummary.msg"
#
#    DEPENDENCIES 에 std_msgs 가 없으면 추가
```

**`inha_interfaces/CMakeLists.txt` 예시:**

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteGraspSrv.srv"
  "srv/MoveitEnable.srv"
  "srv/PlaybackGrasp.srv"
  "srv/JointMoveSrv.srv"
  "msg/CandidateResult.msg"      # 추가
  "msg/RequestSummary.msg"       # 추가
  DEPENDENCIES
    geometry_msgs
    std_msgs                      # 추가 (없으면)
)
```

**`inha_interfaces/package.xml` 추가 의존성:**

```xml
<depend>std_msgs</depend>
```

### Step 2 — `inha_interfaces` 빌드

```bash
cd <ros2_ws>
colcon build --packages-select inha_interfaces
source install/setup.bash

# 확인
ros2 interface show inha_interfaces/msg/RequestSummary
ros2 interface show inha_interfaces/msg/CandidateResult
ros2 interface show inha_interfaces/srv/ExecuteGraspSrv
```

### Step 3 — `inha_control_client` 패키지 업데이트

```bash
cd <ros2_ws>/src/inha_control_client

# 1) 신규 v2 action server 추가
cp <copy_ws>/inha_control_client/src/rby1_action_server_bio_ik_v2.cpp ./src/

# 2) manager 수정사항 적용 (request_id 생성 + 전파)
cp <copy_ws>/inha_control_client/src/rby1_planner_client.cpp ./src/

# 3) prefilter 수정사항 적용 (request_id 전파)
cp <copy_ws>/inha_control_client/src/grasp_prefilter_node.cpp ./src/

# 4) CMakeLists.txt 수정사항 적용
cp <copy_ws>/inha_control_client/CMakeLists.txt ./
```

### Step 4 — `inha_control_client` 빌드

```bash
cd <ros2_ws>
colcon build --packages-select inha_control_client
source install/setup.bash
```

---

## 3. 실행

### 기존 v1 사용 시 (변경 없음)

```bash
ros2 launch inha_control_client rby1_action_server.launch.py
```

### v2 사용 시 (디버깅 활성화)

기존 launch 파일에서 노드 이름만 바꾸거나 직접 실행:

```bash
ros2 run inha_control_client rby1_action_server_v2
```

또는 launch 파일을 복사해서 v2 버전 만들기:

```bash
cp <ros2_ws>/src/inha_control_client/launch/rby1_action_server.launch.py \
   <ros2_ws>/src/inha_control_client/launch/rby1_action_server_v2.launch.py
# 안에서 'rby1_action_server' → 'rby1_action_server_v2' 로 수정
```

---

## 4. 진단 데이터 기록

### 실험 중 rosbag 기록

```bash
ros2 bag record -o run_$(date +%Y%m%d_%H%M%S) \
  --storage mcap \
  /joint_states \
  /tf /tf_static \
  /robot_description \
  /planning_scene \
  /grasp_poses \
  /detection/object_pcd \
  /grasp_candidates \
  /filtered_grasp_candidates \
  /discarded_grasp_candidates \
  /grasp_prefilter/accepted \
  /grasp_prefilter/rejected \
  /manipulation/debug/request_summary \
  /manipulation/debug/candidate_markers \
  /rosout
```

`--storage mcap` 옵션은 ROS2 humble 이상에서 mcap storage plugin 이 설치되어
있어야 합니다. 없으면 다음으로 설치:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

### 트리거

```bash
ros2 service call /manipulation/moveit/enable inha_interfaces/srv/MoveitEnable \
  "{arm_id: 'right', pose_source: 'graspgen'}"
```

이 호출 시 manager 가 request_id 를 생성하고, prefilter 와 action_server 가
같은 request_id 를 공유합니다. action_server 가 종료될 때 RequestSummary
와 MarkerArray 를 발행하여 rosbag 에 기록됩니다.

---

## 5. Foxglove 분석

### 설치

PC 에서:

1. https://foxglove.dev/download 에서 Foxglove Studio 다운로드
2. 설치 후 실행

### MCAP 파일 열기

1. `Open local file...` → `run_*.mcap` 선택
2. 자동으로 robot_description 을 인식하고 URDF 로딩 시도

### 추천 패널 구성

| 패널 | 토픽 / 설정 |
|------|------------|
| **3D** | TF root: `base_nav`<br>Show: URDF (joint_states), TF, `/grasp_poses` (PoseArray), `/manipulation/debug/candidate_markers` (MarkerArray), `/planning_scene` (PlanningScene) |
| **Raw Messages** | `/manipulation/debug/request_summary` — `candidates[]` 펼쳐서 후보별 final_stage 확인 |
| **Plot** | `/manipulation/debug/request_summary` → `n_input`, `n_after_approach_filter`, `n_realgrasp_ik_ok`, `n_pregrasp_ik_ok`, `n_step1_plan_ok`, `n_step2_cartesian_ok` 시계열 |
| **Log** | `/rosout` — RCL 로그 검색 |

### 색상 의미 (3D 패널의 candidate markers)

| 색상 | final_stage | 의미 |
|------|-------------|------|
| 🟢 Green | `winner_executed` | 최종 선택되어 실행됨 |
| 🟠 Orange | `step2_low_coverage` / `step2_misaligned` / `step2_exec_failed` | Step1 까진 OK, Step2 에서 실패 |
| 🟡 Yellow | `step1_plan_failed` / `step1_exec_failed` | IK 까진 OK, Step1 플래닝/실행 실패 |
| 🔴 Red | `realgrasp_ik_failed` / `pregrasp_ik_failed` / `ik_imprecise` | IK 단계 실패 |
| ⚪ Gray | `approach_path_blocked` | 개선 1 (사전 필터) 에서 차단 |
| ⚫ Dark gray | `not_attempted` | 상위 후보 성공으로 시도되지 않음 |

---

## 6. 검증 체크리스트

v2 개선이 의도대로 작동하는지 확인할 항목:

- [ ] **개선 1 작동 확인**: Foxglove 3D 에서 회색 마커가 실제로 막혀 있는 경로 (테이블/물체 뒤) 인지 시각적 확인. 회색 마커 발생 비율이 0%면 필터가 작동하지 않는 것.
- [ ] **개선 2 작동 확인**: `RequestSummary.candidates[].q_realgrasp_pregrasp_distance_rad` 값이 작은지 (대략 < 0.3 rad 이면 의도대로). 큰 값이 자주 나오면 IK seed 가 효과 없음.
- [ ] **단계별 drop-off**: Plot 패널에서 `n_*` 시리즈를 비교. 어느 단계에서 가장 많이 떨어지는지 확인.
- [ ] **Step2 성공률**: `n_step2_cartesian_ok / n_pregrasp_ik_ok` 비율을 v1 vs v2 비교 (별도 실행).
- [ ] **request_id 전파**: 각 토픽의 `request_id` 가 동일한지 확인.

---

## 7. 추후 개선 가능 영역

(이번 작업 범위 외)

- **manager 진단 발행**: 현재는 request_id 만 전파. 입력 단계에서 reachability map reject 된 후보를 별도 토픽으로 publish 하면 prefilter 입력 전 단계까지 추적 가능.
- **prefilter 진단 발행**: rigid_block, camera_collision rejection 을 candidate 별 메시지로 publish. action_server 의 RequestSummary 가 prefilter 정보까지 통합하려면 별도 aggregator 노드 필요.
- **Foxglove 커스텀 패널**: candidate 별 sortable table panel 을 React extension 으로 작성하면 더 편리.

---

## 8. 트러블슈팅

### `inha_interfaces/msg/RequestSummary` not found

→ Step 2 빌드 단계에서 누락. `colcon build --packages-select inha_interfaces` 후 `source install/setup.bash`.

### v2 빌드 시 `visualization_msgs` 헤더 없음

→ `inha_control_client/CMakeLists.txt` 의 v2 타겟에 `visualization_msgs` 의존성이 추가되어 있는지 확인.

### request_id 가 빈 문자열로 나옴

→ manager 의 `generateRequestId()` 가 호출되는지 확인. `cleanly` 사용해서 빌드되었는지 (`ros2 run inha_control_client rby1_grasp_manager` 로그 확인).

### MCAP 기록 안 됨

→ `sudo apt install ros-humble-rosbag2-storage-mcap` 또는 `--storage sqlite3` 로 fallback (Foxglove 가 sqlite3 도 읽음).
