# inha_interfaces 패키지 변경 안내

이 디렉토리의 파일을 robot PC 의 `inha_interfaces` ROS2 패키지에 적용해야
새로 추가된 진단 메시지가 빌드/배포됩니다.

## 1. 새 메시지 파일 추가

```
inha_interfaces/
  msg/
    CandidateResult.msg     ← 신규 (이 디렉토리에서 복사)
    RequestSummary.msg      ← 신규 (이 디렉토리에서 복사)
```

## 2. 기존 srv 파일 수정

`inha_interfaces/srv/ExecuteGraspSrv.srv` 의 Request 부분 (`---` 위) 에
다음 한 줄을 추가합니다:

```
string request_id
```

자세한 내용은 `srv/ExecuteGraspSrv.srv.PATCH` 참조.

## 3. CMakeLists.txt 수정

`rosidl_generate_interfaces(${PROJECT_NAME} ...)` 블록의 파일 목록에
새 메시지 두 개를 추가합니다:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... 기존 srv 들 ...
  "msg/CandidateResult.msg"      # 신규
  "msg/RequestSummary.msg"       # 신규
  DEPENDENCIES
    std_msgs                      # 신규 의존성
    geometry_msgs                 # 기존
)
```

`std_msgs` 가 의존성에 없다면 추가해야 합니다 (Header 사용을 위해).

## 4. package.xml 수정

`std_msgs` 의존성이 없다면 추가:

```xml
<depend>std_msgs</depend>
```

## 5. 빌드

```bash
cd ~/ros2_ws  # robot PC 의 ROS2 workspace
colcon build --packages-select inha_interfaces
source install/setup.bash
```

빌드 후 다음 명령으로 확인:

```bash
ros2 interface show inha_interfaces/msg/RequestSummary
ros2 interface show inha_interfaces/msg/CandidateResult
ros2 interface show inha_interfaces/srv/ExecuteGraspSrv
```
