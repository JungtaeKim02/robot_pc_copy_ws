cmake_minimum_required(VERSION 3.8)
project(rby1_control_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
# [중요] 액션 인터페이스 패키지 찾기
find_package(inha_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# grasp_prefilter_node 가 사용하는 추가 의존성
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# bio_ik는 kinematics.yaml에서 런타임 플러그인으로 로드되므로 find_package 불필요

include_directories(include)

# 1. Action Server - BioIK 버전 (메인)
#    bio_ik는 런타임 kinematics 플러그인이므로 컴파일 의존성 없음
add_executable(rby1_action_server src/rby1_action_server_bio_ik.cpp)
ament_target_dependencies(rby1_action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  inha_interfaces
)

# 1-2. Action Server - BioIK v2 (개선판 + 진단 시각화)
#    개선 1: approach 경로 collision 사전 필터
#    개선 2: real-grasp IK → seed → pre-grasp IK
#    추가: RequestSummary + MarkerArray 진단 토픽 발행 (Foxglove 시각화)
add_executable(rby1_action_server_v2 src/rby1_action_server_bio_ik_v2.cpp)
ament_target_dependencies(rby1_action_server_v2
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_core
  inha_interfaces
)
target_link_libraries(rby1_action_server_v2 ${OCTOMAP_LIBRARIES})
target_include_directories(rby1_action_server_v2 PRIVATE ${OCTOMAP_INCLUDE_DIRS})

# 1-1. Action Server - KDL 버전 (백업)
add_executable(rby1_action_server_kdl src/rby1_action_server.cpp)
ament_target_dependencies(rby1_action_server_kdl
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  inha_interfaces
)

# 2. Grasp Manager (사용자 UI 및 클라이언트) — 다중 pose source 지원
add_executable(rby1_grasp_manager src/rby1_planner_client.cpp)
ament_target_dependencies(rby1_grasp_manager
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  inha_interfaces
)

# 3. Grasp Prefilter Node
#    yaw flip 복제 + rigid-block + 카메라 콜리전 검사 분리 노드.
#    manager → prefilter → action_server 체인의 가운데 단계.
add_executable(grasp_prefilter_node src/grasp_prefilter_node.cpp)
ament_target_dependencies(grasp_prefilter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  octomap_msgs
  inha_interfaces
)
target_link_libraries(grasp_prefilter_node ${OCTOMAP_LIBRARIES})
target_include_directories(grasp_prefilter_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})

install(TARGETS
  rby1_action_server
  rby1_action_server_v2
  rby1_action_server_kdl
  rby1_grasp_manager
  grasp_prefilter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
