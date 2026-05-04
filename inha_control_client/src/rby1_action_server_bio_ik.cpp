#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// Rigid-block pre-filter: octomap point query
#include <geometric_shapes/shapes.h>
#include <octomap/OcTree.h>

// Attached collision object: 잡은 객체를 로봇 몸체 일부로 인식
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// TF2: world→base_nav 등 URDF 외부 프레임 변환 조회
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// bio_ik는 kinematics.yaml에서 런타임 플러그인으로 로드됨
// PickNikRobotics/bio_ik (ros2 브랜치)는 표준 MoveIt API + cost function 사용
// #include <bio_ik/bio_ik.h> 는 불필요

#include "inha_interfaces/srv/execute_grasp_srv.hpp"
#include "inha_interfaces/srv/playback_grasp.hpp"
#include "inha_interfaces/srv/joint_move_srv.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <random>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <typeinfo>
#include <sys/stat.h>

using ExecuteGraspSrv = inha_interfaces::srv::ExecuteGraspSrv;
using PlaybackGrasp = inha_interfaces::srv::PlaybackGrasp;
using JointMoveSrv = inha_interfaces::srv::JointMoveSrv;

class Rby1ActionServer : public rclcpp::Node
{
public:
  explicit Rby1ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rby1_grasp_action_server", options)
  {
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // ⚠️ 서비스 이름 변경: prefilter 노드 도입에 따라 "execute_grasp_service"
    //    는 grasp_prefilter_node 가 차지한다. 본 action_server 는 prefilter 가
    //    호출하는 다운스트림 서비스로 재배치된다.
    //      manager → /execute_grasp_service(prefilter)
    //              → /execute_grasp_planning(action_server, 본 서비스)
    service_server_ = this->create_service<ExecuteGraspSrv>(
      "execute_grasp_planning",
      std::bind(&Rby1ActionServer::handle_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_
    );

    playback_server_ = this->create_service<PlaybackGrasp>(
      "playback_grasp_service",
      std::bind(&Rby1ActionServer::handle_playback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_
    );

    // Joint-space 목표값 기반 collision-aware planning & execution
    joint_move_server_ = this->create_service<JointMoveSrv>(
      "joint_move_service",
      std::bind(&Rby1ActionServer::handle_joint_move, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_
    );

    get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    apply_planning_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");

    // Grasp pose publisher (PoseArray: [0]=left, [1]=right)
    grasp_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/grasp_poses", rclcpp::QoS(1).transient_local());

    // 초기화: NaN pose 2개 (left, right) — NaN = 유효하지 않음
    grasp_poses_msg_.header.frame_id = "base";
    grasp_poses_msg_.poses.resize(2);
    auto make_nan_pose = []() {
      geometry_msgs::msg::Pose p;
      p.position.x = p.position.y = p.position.z = std::numeric_limits<double>::quiet_NaN();
      p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = std::numeric_limits<double>::quiet_NaN();
      return p;
    };
    grasp_poses_msg_.poses[0] = make_nan_pose();
    grasp_poses_msg_.poses[1] = make_nan_pose();
    left_grasp_valid_ = false;
    right_grasp_valid_ = false;

    // Object point cloud 구독 (Attached Collision Object 생성용)
    object_pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/detection/object_pcd",
      rclcpp::QoS(1).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(object_pcd_mutex_);
        cached_object_pcd_ = msg;
        RCLCPP_DEBUG(get_logger(), "[PCD] Cached object point cloud (%u points, frame=%s)",
                     msg->width * msg->height, msg->header.frame_id.c_str());
      });

    // TF2 buffer: URDF 외부 프레임(world 등) 변환 조회용
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "🚀Rby1 Grasp Action Server Created. Waiting for MoveIt...");
  }

  void init_moveit()
  {
    try {
        move_group_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rby1_right_arm");
        move_group_right_->setPlanningTime(3.0);
        move_group_right_->setMaxVelocityScalingFactor(0.1);
        move_group_right_->setMaxAccelerationScalingFactor(0.1);
        RCLCPP_INFO(get_logger(), "MoveIt 'rby1_right_arm' Loaded.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load 'rby1_right_arm': %s", e.what());
    }

    try {
        move_group_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rby1_left_arm");
        move_group_left_->setPlanningTime(3.0);
        move_group_left_->setMaxVelocityScalingFactor(0.1);
        move_group_left_->setMaxAccelerationScalingFactor(0.1);
        RCLCPP_INFO(get_logger(), "MoveIt 'rby1_left_arm' Loaded.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load 'rby1_left_arm': %s", e.what());
    }

    try {
        move_group_right_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rby1_right_arm_torso");
        move_group_right_torso_->setPlanningTime(3.0);
        move_group_right_torso_->setMaxVelocityScalingFactor(0.1);
        move_group_right_torso_->setMaxAccelerationScalingFactor(0.1);
        RCLCPP_INFO(get_logger(), "MoveIt 'rby1_right_arm_torso' Loaded.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load 'rby1_right_arm_torso': %s", e.what());
    }

    try {
        move_group_left_torso_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rby1_left_arm_torso");
        move_group_left_torso_->setPlanningTime(3.0);
        move_group_left_torso_->setMaxVelocityScalingFactor(0.1);
        move_group_left_torso_->setMaxAccelerationScalingFactor(0.1);
        RCLCPP_INFO(get_logger(), "MoveIt 'rby1_left_arm_torso' Loaded.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load 'rby1_left_arm_torso': %s", e.what());
    }

    if(move_group_right_) move_group_right_->startStateMonitor(2.0);
    if(move_group_left_) move_group_left_->startStateMonitor(2.0);
    if(move_group_right_torso_) move_group_right_torso_->startStateMonitor(2.0);
    if(move_group_left_torso_) move_group_left_torso_->startStateMonitor(2.0);

    while (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /get_planning_scene service...");
    }
    while (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /apply_planning_scene service...");
    }

    // ==========================================
    // IK solver 진단 로그
    //   각 planning group 이 실제 사용 중인 solver 이름과 DOF 를 출력.
    //   kinematics.yaml 설정이 실제로 로드되었는지 확인용.
    // ==========================================
    auto log_group_solver = [this](moveit::planning_interface::MoveGroupInterface* mg,
                                    const char* label) {
      if (!mg) return;
      try {
        auto cs = mg->getCurrentState();
        if (!cs) {
          RCLCPP_WARN(get_logger(), "[IK Info] %s: Failed to get current state (Robot state not received yet).", label);
          return;
        }
        const auto* jmg = cs->getJointModelGroup(mg->getName());
        if (!jmg) {
          RCLCPP_WARN(get_logger(), "[IK Info] %s: JMG '%s' not found.",
                      label, mg->getName().c_str());
          return;
        }
        const auto solver = jmg->getSolverInstance();
        size_t dof = jmg->getActiveJointModels().size();
        std::string solver_name = solver ? typeid(*solver).name() : std::string("NONE");
        std::string tip_frame = solver ? solver->getTipFrame() : std::string("<none>");
        RCLCPP_INFO(get_logger(),
          "[IK Info] %s: group='%s'  DOF=%zu  solver_type='%s'  tip_frame='%s'",
          label, mg->getName().c_str(), dof, solver_name.c_str(), tip_frame.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[IK Info] %s: query failed: %s", label, e.what());
      }
    };
    log_group_solver(move_group_right_.get(),       "right_arm      ");
    log_group_solver(move_group_left_.get(),        "left_arm       ");
    log_group_solver(move_group_right_torso_.get(), "right_arm_torso");
    log_group_solver(move_group_left_torso_.get(),  "left_arm_torso ");

    RCLCPP_INFO(get_logger(), "MoveIt Interface initialized. Ready.");
  }

private:
  rclcpp::Service<ExecuteGraspSrv>::SharedPtr service_server_;
  rclcpp::Service<PlaybackGrasp>::SharedPtr playback_server_;
  rclcpp::Service<JointMoveSrv>::SharedPtr joint_move_server_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_torso_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_torso_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;

  // 플레이백을 위한 궤적 저장
  std::mutex trajectory_mutex_;
  moveit_msgs::msg::RobotTrajectory stored_step1_trajectory_;
  moveit_msgs::msg::RobotTrajectory stored_step2_trajectory_;
  std::string stored_arm_id_;
  bool trajectory_available_ = false;

  // Grasp pose publisher & storage
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr grasp_pose_pub_;
  geometry_msgs::msg::PoseArray grasp_poses_msg_;  // [0]=left, [1]=right
  geometry_msgs::msg::Pose stored_grasp_pose_;     // 현재 grasp의 EEF pose
  bool left_grasp_valid_ = false;
  bool right_grasp_valid_ = false;

  // Attached collision object: 잡은 객체를 로봇 몸체로 인식
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr object_pcd_sub_;
  std::mutex object_pcd_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr cached_object_pcd_;
  std::string attached_object_id_;  // 비어있으면 부착된 객체 없음

  // TF2 buffer: URDF 외부 프레임(world 등) 변환 조회용
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ==========================================
  // 디버깅 로그 구조체 및 유틸리티
  // ==========================================
  struct IkLogEntry {
    size_t candidate_index;
    bool rigid_block_skip = false;     // rigid-block pre-filter 에 의해 skip 된 경우 true
    std::string rigid_block_detail;    // collision detail (빈 문자열 = 통과)
    size_t precise_solutions;
    double best_ns_cost, best_manip, best_joint;
  };

  struct PlanAttemptLog {
    size_t rank;
    size_t candidate_index;
    std::string step1_result;
    double step2_coverage;
    std::string step2_trim_result;
    double trim_distance;
    double direction_alignment;
    std::string overall_result;
    std::string failure_reason;
  };

  struct BatchLog {
    size_t batch_index;
    std::string candidate_range;
    std::vector<IkLogEntry> ik_results;
    std::vector<PlanAttemptLog> plan_attempts;
  };

  struct TorsoCandidate {
    std::vector<double> angles_deg;
    double cost;
    double cost_reach;
    double cost_dist_sweet;
    double cost_torso_move;
    double avg_dist;
    Eigen::Vector3d shoulder_world;
  };

  struct Phase0Log {
    bool attempted = false;
    bool success = false;
    std::vector<double> current_torso_angles_deg;
    std::vector<double> best_torso_angles_deg;
    double best_cost = 0;
    double cost_reach = 0;
    double cost_manip_inv = 0;
    double cost_dist_sweet = 0;  // (dist - D_SWEET)^2 per-candidate 평균 혹은 dist 최소값
    double cost_torso_move = 0;
    double best_avg_dist = 0;    // 디버그: 선택된 자세에서 후보까지 평균 거리
    double best_shoulder_x = 0;
    double best_shoulder_y = 0;
    double best_shoulder_z = 0;
    size_t total_torso_samples = 0;
    size_t feasible_torso_samples = 0;
    size_t candidates_sampled = 0;
    double elapsed_ms = 0;

    std::vector<TorsoCandidate> top_candidates;
  };

  struct Phase1Log {
    bool attempted = false;
    bool success = false;
    std::vector<double> target_torso_angles_deg;
    size_t trajectory_points = 0;
    double elapsed_ms = 0;
    std::string failure_reason;
  };

  struct GraspLog {
    std::string timestamp;
    std::string arm_id;
    size_t total_candidates = 0;
    Phase0Log phase0;
    Phase1Log phase1;
    std::vector<BatchLog> batches;
    bool final_success = false;
    size_t winning_candidate = 0;
    size_t winning_batch = 0;
    size_t winning_rank = 0;
    double total_elapsed_ms = 0;
  };

  static std::string escapeJson(const std::string& s) {
    std::string out;
    for (char c : s) {
      if (c == '"') out += "\\\"";
      else if (c == '\\') out += "\\\\";
      else out += c;
    }
    return out;
  }

  void writeGraspLog(const GraspLog& log) {
    mkdir("/tmp/grasp_logs", 0755);

    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char fname[128];
    std::strftime(fname, sizeof(fname), "/tmp/grasp_logs/grasp_%Y%m%d_%H%M%S_", std::localtime(&t));
    std::string filepath = std::string(fname) + log.arm_id + ".json";

    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
      RCLCPP_WARN(get_logger(), "[Log] Cannot write to %s", filepath.c_str());
      return;
    }

    auto write_angles = [&ofs](const std::vector<double>& v) {
      ofs << "[";
      for (size_t i = 0; i < v.size(); ++i) {
        ofs << std::fixed << std::setprecision(2) << v[i];
        if (i + 1 < v.size()) ofs << ", ";
      }
      ofs << "]";
    };

    ofs << "{\n";
    ofs << "  \"timestamp\": \"" << log.timestamp << "\",\n";
    ofs << "  \"arm_id\": \"" << log.arm_id << "\",\n";
    ofs << "  \"total_candidates\": " << log.total_candidates << ",\n";
    ofs << "  \"total_elapsed_ms\": " << std::fixed << std::setprecision(1) << log.total_elapsed_ms << ",\n";

    // Phase 0: Torso optimization
    ofs << "  \"phase0_torso_optimization\": {\n";
    ofs << "    \"attempted\": " << (log.phase0.attempted ? "true" : "false") << ",\n";
    ofs << "    \"success\": " << (log.phase0.success ? "true" : "false") << ",\n";
    ofs << "    \"total_torso_samples\": " << log.phase0.total_torso_samples << ",\n";
    ofs << "    \"feasible_torso_samples\": " << log.phase0.feasible_torso_samples << ",\n";
    ofs << "    \"candidates_sampled\": " << log.phase0.candidates_sampled << ",\n";
    ofs << "    \"current_torso_angles_deg\": ";
    write_angles(log.phase0.current_torso_angles_deg); ofs << ",\n";
    ofs << "    \"best_torso_angles_deg\": ";
    write_angles(log.phase0.best_torso_angles_deg); ofs << ",\n";
    ofs << "    \"best_cost\": " << std::fixed << std::setprecision(4) << log.phase0.best_cost << ",\n";
    ofs << "    \"cost_breakdown\": {\n";
    ofs << "      \"reach\": " << log.phase0.cost_reach << ",\n";
    ofs << "      \"manipulability_inv\": " << log.phase0.cost_manip_inv << ",\n";
    ofs << "      \"dist_sweet\": " << log.phase0.cost_dist_sweet << ",\n";
    ofs << "      \"torso_move\": " << log.phase0.cost_torso_move << "\n";
    ofs << "    },\n";
    ofs << "    \"best_avg_dist_m\": " << std::setprecision(4) << log.phase0.best_avg_dist << ",\n";
    ofs << "    \"best_shoulder_world\": ["
        << log.phase0.best_shoulder_x << ", "
        << log.phase0.best_shoulder_y << ", "
        << log.phase0.best_shoulder_z << "],\n";
    ofs << "    \"elapsed_ms\": " << std::fixed << std::setprecision(1) << log.phase0.elapsed_ms << "\n";
    ofs << "  },\n";

    // Phase 1: Torso move
    ofs << "  \"phase1_torso_move\": {\n";
    ofs << "    \"attempted\": " << (log.phase1.attempted ? "true" : "false") << ",\n";
    ofs << "    \"success\": " << (log.phase1.success ? "true" : "false") << ",\n";
    ofs << "    \"target_torso_angles_deg\": ";
    write_angles(log.phase1.target_torso_angles_deg); ofs << ",\n";
    ofs << "    \"trajectory_points\": " << log.phase1.trajectory_points << ",\n";
    ofs << "    \"elapsed_ms\": " << std::fixed << std::setprecision(1) << log.phase1.elapsed_ms << ",\n";
    ofs << "    \"failure_reason\": \"" << escapeJson(log.phase1.failure_reason) << "\"\n";
    ofs << "  },\n";

    ofs << "  \"batches\": [\n";
    for (size_t bi = 0; bi < log.batches.size(); ++bi) {
      const auto& b = log.batches[bi];
      ofs << "    {\n";
      ofs << "      \"batch_index\": " << b.batch_index << ",\n";
      ofs << "      \"candidate_range\": \"" << b.candidate_range << "\",\n";

      ofs << "      \"ik_results\": [\n";
      for (size_t ii = 0; ii < b.ik_results.size(); ++ii) {
        const auto& ik = b.ik_results[ii];
        ofs << "        {\"candidate_index\": " << ik.candidate_index
            << ", \"rigid_block_skip\": " << (ik.rigid_block_skip ? "true" : "false");
        if (!ik.rigid_block_detail.empty()) {
          ofs << ", \"rigid_block_detail\": \"" << ik.rigid_block_detail << "\"";
        }
        ofs << ", \"precise_solutions\": " << ik.precise_solutions
            << ", \"best_ns_cost\": " << std::setprecision(4) << ik.best_ns_cost
            << ", \"best_manip\": " << ik.best_manip
            << ", \"best_joint\": " << ik.best_joint << "}";
        if (ii + 1 < b.ik_results.size()) ofs << ",";
        ofs << "\n";
      }
      ofs << "      ],\n";

      ofs << "      \"plan_attempts\": [\n";
      for (size_t pi = 0; pi < b.plan_attempts.size(); ++pi) {
        const auto& pa = b.plan_attempts[pi];
        ofs << "        {\"rank\": " << pa.rank
            << ", \"candidate_index\": " << pa.candidate_index
            << ", \"step1_result\": \"" << pa.step1_result << "\""
            << ", \"step2_coverage\": " << std::setprecision(1) << pa.step2_coverage
            << ", \"step2_trim_result\": \"" << pa.step2_trim_result << "\""
            << ", \"trim_distance\": " << std::setprecision(4) << pa.trim_distance
            << ", \"direction_alignment\": " << std::setprecision(3) << pa.direction_alignment
            << ", \"overall_result\": \"" << pa.overall_result << "\""
            << ", \"failure_reason\": \"" << escapeJson(pa.failure_reason) << "\"}";
        if (pi + 1 < b.plan_attempts.size()) ofs << ",";
        ofs << "\n";
      }
      ofs << "      ]\n";

      ofs << "    }";
      if (bi + 1 < log.batches.size()) ofs << ",";
      ofs << "\n";
    }
    ofs << "  ],\n";

    ofs << "  \"final_result\": {\n";
    ofs << "    \"success\": " << (log.final_success ? "true" : "false") << ",\n";
    ofs << "    \"winning_candidate\": " << log.winning_candidate << ",\n";
    ofs << "    \"winning_batch\": " << log.winning_batch << ",\n";
    ofs << "    \"winning_rank\": " << log.winning_rank << "\n";
    ofs << "  }\n";
    ofs << "}\n";

    ofs.close();
    RCLCPP_INFO(get_logger(), "📝 [Log] Written to %s", filepath.c_str());
  }

  std::string currentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    return std::string(buf);
  }

  // ==========================================
  // [Deprecated — 호출되지 않음]
  //   본 함수는 grasp_prefilter_node 로 이전됐다. handle_service 는 더 이상
  //   이를 호출하지 않으며, 백업/디버그 목적으로 본체는 보존만 한다.
  //   삭제하지 않은 이유: 노드 분리 직후 회귀 검증 시 빠른 롤백을 위해.
  // Rigid-block pre-filter: IK 이전에 고정 링크 블록의 octomap 충돌 검사
  //
  // EE world pose 가 주어지면, arm_5 → arm_6 → ee → fingers 링크는
  // IK 해와 무관하게 world 위치가 결정된다 (arm_6 은 Z 축 revolute 이고
  // arm_5 cylinder 는 Z 대칭이므로 회전해도 collision envelope 불변).
  //
  // 이 블록에 대한 octomap 점 쿼리로 "어떤 IK 해를 구해도 반드시 충돌"하는
  // 후보를 IK 연산 없이 O(1) 에 거부할 수 있다.
  //
  // 반환: true = 충돌 (reject), false = 통과
  // ==========================================
  bool checkRigidBlockCollision(
      const geometry_msgs::msg::Pose& ee_pose_world,
      const planning_scene::PlanningScenePtr& scene,
      size_t cand_idx,
      std::string& detail_out)
  {
    detail_out.clear();
    if (!scene) return false;  // scene 없으면 필터 비활성

    // ---- Octomap 추출 ----
    const auto& world = scene->getWorld();
    auto octomap_obj = world->getObject("<octomap>");
    if (!octomap_obj || octomap_obj->shapes_.empty()) {
      RCLCPP_WARN_ONCE(get_logger(), "[RigidBlock] No octomap in planning scene, skipping pre-filter.");
      return false;
    }

    auto octree_shape = std::dynamic_pointer_cast<const shapes::OcTree>(octomap_obj->shapes_[0]);
    if (!octree_shape || !octree_shape->octree) {
      RCLCPP_WARN_ONCE(get_logger(), "[RigidBlock] OcTree shape cast failed.");
      return false;
    }
    const auto& octree = octree_shape->octree;
    // octomap 의 world frame 변환 (보통 identity 이지만 일반성 확보)
    Eigen::Isometry3d octomap_tf = Eigen::Isometry3d::Identity();
    if (!octomap_obj->shape_poses_.empty()) {
      octomap_tf = octomap_obj->shape_poses_[0];
    }
    Eigen::Isometry3d octomap_tf_inv = octomap_tf.inverse();

    // ---- EE world transform 구성 ----
    Eigen::Isometry3d T_ee = Eigen::Isometry3d::Identity();
    T_ee.translation() = Eigen::Vector3d(
        ee_pose_world.position.x, ee_pose_world.position.y, ee_pose_world.position.z);
    T_ee.linear() = Eigen::Quaterniond(
        ee_pose_world.orientation.w, ee_pose_world.orientation.x,
        ee_pose_world.orientation.y, ee_pose_world.orientation.z).toRotationMatrix();

    // ---- 샘플 포인트 정의 (EE local frame) ----
    // URDF 기준:
    //   arm_5 cylinder: center at EE local [0, 0, +0.0261], R=0.05, half-L=0.09
    //     → Z spans: -0.0639 ~ +0.1161
    //   finger_r1: collision box center at EE local [+0.05, 0, -0.10525]
    //   finger_r2: collision box center at EE local [-0.05, 0, -0.10525]
    //     → finger tip Z ≈ -0.135, finger base Z ≈ -0.075
    struct SamplePt { double x, y, z; const char* label; };
    static const SamplePt samples[] = {
      // --- Finger tips (가장 앞단, 테이블/물체와 먼저 충돌) ---
      { 0.05,  0.0, -0.135, "finger1_tip"},
      {-0.05,  0.0, -0.135, "finger2_tip"},
      // --- Finger centers ---
      { 0.05,  0.0, -0.105, "finger1_mid"},
      {-0.05,  0.0, -0.105, "finger2_mid"},
      // --- Finger bases ---
      { 0.05,  0.0, -0.075, "finger1_base"},
      {-0.05,  0.0, -0.075, "finger2_base"},
      // --- EE center (grip center) ---
      { 0.0,   0.0, -0.03,  "ee_low"},
      { 0.0,   0.0,  0.0,   "ee_center"},
      // --- arm_5 cylinder: axis samples ---
      { 0.0,   0.0, -0.06,  "arm5_bot"},
      { 0.0,   0.0,  0.0,   "arm5_z0"},
      { 0.0,   0.0,  0.03,  "arm5_z1"},
      { 0.0,   0.0,  0.06,  "arm5_z2"},
      { 0.0,   0.0,  0.09,  "arm5_z3"},
      { 0.0,   0.0,  0.116, "arm5_top"},
      // --- arm_5 cylinder: radial samples (R=0.045, 약간 내측으로 마진) ---
      { 0.045,  0.0,   0.0,   "arm5_r+x_0"},
      {-0.045,  0.0,   0.0,   "arm5_r-x_0"},
      { 0.0,    0.045, 0.0,   "arm5_r+y_0"},
      { 0.0,   -0.045, 0.0,   "arm5_r-y_0"},
      { 0.045,  0.0,   0.06,  "arm5_r+x_1"},
      {-0.045,  0.0,   0.06,  "arm5_r-x_1"},
      { 0.0,    0.045, 0.06,  "arm5_r+y_1"},
      { 0.0,   -0.045, 0.06,  "arm5_r-y_1"},
      { 0.045,  0.0,   0.116, "arm5_r+x_t"},
      {-0.045,  0.0,   0.116, "arm5_r-x_t"},
    };
    constexpr size_t N_SAMPLES = sizeof(samples) / sizeof(samples[0]);

    // ---- 각 샘플 포인트를 world → octomap frame 변환 후 occupancy 쿼리 ----
    size_t n_occupied = 0;
    std::string first_hit_label;
    for (size_t s = 0; s < N_SAMPLES; ++s) {
      Eigen::Vector3d pt_local(samples[s].x, samples[s].y, samples[s].z);
      Eigen::Vector3d pt_world = T_ee * pt_local;
      Eigen::Vector3d pt_oct = octomap_tf_inv * pt_world;

      auto node = octree->search(pt_oct.x(), pt_oct.y(), pt_oct.z());
      if (node && octree->isNodeOccupied(node)) {
        n_occupied++;
        if (first_hit_label.empty()) {
          first_hit_label = samples[s].label;
        }
      }
    }

    if (n_occupied > 0) {
      char buf[256];
      snprintf(buf, sizeof(buf),
        "rigid-block %zu/%zu pts occupied (first: %s)",
        n_occupied, N_SAMPLES, first_hit_label.c_str());
      detail_out = buf;
      RCLCPP_INFO(get_logger(),
        "[RigidBlock] Cand %zu: COLLISION — %s", cand_idx, buf);
      return true;
    }

    RCLCPP_DEBUG(get_logger(),
      "[RigidBlock] Cand %zu: CLEAR (%zu pts checked)", cand_idx, N_SAMPLES);
    return false;
  }

  // ==========================================
  // Phase 0: Torso 최적 자세 탐색 (Arm Base Placement)
  //
  // torso_3/4/5 ±30° 샘플링 → 각 샘플에서 arm Jacobian 기반
  // 비용 함수 평가 → 최소 비용 조합 선택.
  //
  // f(θ_t) = w1·C_reach + w2·(1/w_d²) + w3·C_align + w4·C_torso_move
  //
  // world_poses: planning frame 기준 grasp 후보 pose (Phase 1 전에 캡처된 상태)
  // active_group: arm-only MoveGroup (rby1_right_arm 등) — arm JMG 확보용
  // arm_base_link: "link_right_arm_0" 또는 "link_left_arm_0"
  // ee_link: "ee_right" 또는 "ee_left"
  // ==========================================
  Phase0Log runPhase0TorsoOptimization(
      moveit::planning_interface::MoveGroupInterface* active_group,
      moveit::planning_interface::MoveGroupInterface* active_group_torso,
      const std::vector<geometry_msgs::msg::Pose>& world_poses,
      const std::string& arm_base_link,
      const std::string& ee_link)
  {
    auto t_start = std::chrono::steady_clock::now();
    Phase0Log result;
    result.attempted = true;

    if (world_poses.empty()) {
      result.elapsed_ms = 0;
      return result;
    }

    // 현재 상태 확보
    auto ref_state = active_group_torso->getCurrentState();
    ref_state->update();
    const auto* arm_jmg = ref_state->getJointModelGroup(active_group->getName());
    if (!arm_jmg) {
      RCLCPP_WARN(get_logger(), "[Phase0] arm JMG not found: %s", active_group->getName().c_str());
      return result;
    }

    // 현재 torso 각도
    const double cur_t3 = ref_state->getVariablePosition("torso_3");
    const double cur_t4 = ref_state->getVariablePosition("torso_4");
    const double cur_t5 = ref_state->getVariablePosition("torso_5");
    result.current_torso_angles_deg = {
      cur_t3 * 180.0 / M_PI,
      cur_t4 * 180.0 / M_PI,
      cur_t5 * 180.0 / M_PI
    };

    // 샘플링 범위 (라디안)
    const double DEG = M_PI / 180.0;
    std::vector<double> t3_samples, t4_samples, t5_samples;
    for (double d = -20.0; d <= 20.0 + 1e-6; d += 5.0)  t3_samples.push_back(cur_t3 + d * DEG);
    for (double d = -20.0; d <= 20.0 + 1e-6; d += 10.0) t4_samples.push_back(cur_t4 + d * DEG);
    for (double d = -20.0; d <= 20.0 + 1e-6; d += 10.0) t5_samples.push_back(cur_t5 + d * DEG);

    result.total_torso_samples = t3_samples.size() * t4_samples.size() * t5_samples.size();

    // 관절 한계 체크 헬퍼
    auto in_bounds = [&](const std::string& name, double val) -> bool {
      const auto* jm = ref_state->getRobotModel()->getJointModel(name);
      if (!jm) return false;
      const auto& vb = jm->getVariableBounds();
      if (vb.empty()) return true;
      const auto& b = vb[0];
      if (!b.position_bounded_) return true;
      return val >= b.min_position_ && val <= b.max_position_;
    };

    // 후보의 접근 방향과 위치를 미리 추출
    struct Cand {
      Eigen::Vector3d p_world;
      Eigen::Vector3d v_app_world;  // 단위 벡터 (local -Z in world)
    };
    std::vector<Cand> cands;
    cands.reserve(world_poses.size());
    for (const auto& p : world_poses) {
      Cand c;
      c.p_world = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
      Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
      c.v_app_world = -q.toRotationMatrix().col(2);
      double n = c.v_app_world.norm();
      if (n > 1e-9) c.v_app_world /= n;
      cands.push_back(c);
    }

    // 비용 가중치
    // NOTE: 순수 유클리디안(avg_dist) 최소화는 "무조건 가까이" 를 선호해
    //       일부 후보가 너무 접혀 IK 실패 / arm_5 관절 한계 근접 을 유발했다.
    //       대신 "조작성이 좋은 거리 스윗스팟" 을 벌점화한다:
    //         C_dist_sweet = mean_topK ( (dist_k - D_IDEAL)^2 )
    //
    //       [D_IDEAL 튜닝]
    //         - 이전 성공 Cand 40 grasp dist=0.441 m
    //         - 이전 성공 Cand 14 grasp dist=0.484 m
    //         - 현재 world_poses 는 grasp 에서 +12 cm PREGRASP_STANDOFF 된 pose.
    //           radial 성분이 ~8–10 cm 증가 → pregrasp 공간의 sweet spot ≈ 0.56–0.60 m.
    //         - 이론: 7-DOF arm (L≈0.85 m) elbow 90° → wrist ~0.5 m (grasp 기준)
    //       따라서 D_IDEAL = 0.60 m.
    //
    //       [Top-K 집계 (dispersion 대응)]
    //         72 candidate 를 전부 평균하면 "평균 거리가 D 인 분산된 분포" 를
    //         선호하게 되어, 실제로 sweet spot 에 오는 candidate 는 거의 없고
    //         모두 애매한 거리에 놓이는 compromise shoulder 가 뽑힌다.
    //         "적어도 K 개 만이라도 완벽히 sweet spot" 을 선호하도록,
    //         (d-D)² 중 가장 작은 K 개만 평균한다. K = 6 (batch 2개분).
    //
    //       정렬(u·a) 항은 추가하지 않음:
    //         - Phase 0 는 arm IK 를 풀지 않아 Jacobian 정보가 없음
    //         - 과거 w3_align 은 shoulder 를 R_MAX 로 몰아붙이는 부작용
    //         - 거리 sweet-spot 한 항이 두 극단(가까움/멀음)을 동시 해결
    const double w1_reach         = 50.0;    // 도달 한계 초과: 강한 페널티
    const double w2_dist_sweet    = 100.0;   // (dist - D_IDEAL)^2 top-K 평균
    const double w3_torso         = 1.0;     // torso 이동량
    const double R_MAX            = 0.85;    // arm 도달 반경 (m)
    const double D_IDEAL          = 0.60;    // 조작성 sweet spot 거리 (m, pregrasp 12cm standoff 기준)
                                              // 성공 사례 grasp dist 0.44-0.48m + standoff 0.12m 투영 ≈ 0.56-0.60m
    const size_t K_BEST           = 6;       // top-K: 상위 K개 후보만 평균

    auto eval_state = std::make_shared<moveit::core::RobotState>(*ref_state);
    const moveit::core::LinkModel* arm_base_lm = eval_state->getLinkModel(arm_base_link);
    const moveit::core::LinkModel* ee_lm = eval_state->getLinkModel(ee_link);
    if (!arm_base_lm || !ee_lm) {
      RCLCPP_WARN(get_logger(), "[Phase0] link model not found (arm_base=%s, ee=%s)",
                  arm_base_link.c_str(), ee_link.c_str());
      return result;
    }

    std::vector<TorsoCandidate> all_cands;

    // [Phase0] 시작 상태에서 어깨 world 위치 로그 (디버그용)
    {
      const Eigen::Isometry3d& T_base0 = ref_state->getGlobalLinkTransform(arm_base_lm);
      Eigen::Vector3d sh0 = T_base0.translation();
      RCLCPP_INFO(get_logger(),
        "[Phase0] Start: torso=[%.1f°, %.1f°, %.1f°]  shoulder(%s) world=[%.3f, %.3f, %.3f]  "
        "candidates=%zu  grid=%zu samples",
        result.current_torso_angles_deg[0], result.current_torso_angles_deg[1],
        result.current_torso_angles_deg[2],
        arm_base_link.c_str(), sh0.x(), sh0.y(), sh0.z(),
        world_poses.size(), result.total_torso_samples);

      // 첫 번째 후보까지의 거리 참고 값
      if (!cands.empty()) {
        Eigen::Vector3d d0 = cands[0].p_world - sh0;
        RCLCPP_INFO(get_logger(),
          "[Phase0] Cand[0] world=[%.3f, %.3f, %.3f]  dist_from_shoulder=%.3f m (R_MAX=%.2f)",
          cands[0].p_world.x(), cands[0].p_world.y(), cands[0].p_world.z(),
          d0.norm(), R_MAX);
      }
    }

    for (double t3 : t3_samples) {
      if (!in_bounds("torso_3", t3)) continue;
      for (double t4 : t4_samples) {
        if (!in_bounds("torso_4", t4)) continue;
        for (double t5 : t5_samples) {
          if (!in_bounds("torso_5", t5)) continue;

          eval_state->setVariablePosition("torso_3", t3);
          eval_state->setVariablePosition("torso_4", t4);
          eval_state->setVariablePosition("torso_5", t5);
          eval_state->update();

          // 어깨(arm_base) world 변환
          const Eigen::Isometry3d& T_base = eval_state->getGlobalLinkTransform(arm_base_lm);
          const Eigen::Vector3d t_shoulder = T_base.translation();

          double sum_reach = 0, sum_dist = 0;
          std::vector<double> dd2_list;  // top-K 집계용
          dd2_list.reserve(cands.size());
          size_t n_valid = 0;

          for (const auto& c : cands) {
            Eigen::Vector3d d = c.p_world - t_shoulder;
            double dist = d.norm();
            double c_reach = std::max(0.0, dist - R_MAX);

            // 너무 멀면 이 후보는 스킵 (torso 이동으로도 해결 불가)
            if (c_reach > 0.3) continue;

            // sweet-spot 편차 제곱 (top-K 만 사후 집계)
            double dd = dist - D_IDEAL;
            dd2_list.push_back(dd * dd);

            sum_reach += c_reach;
            sum_dist  += dist;
            n_valid++;
            result.candidates_sampled++;
          }

          if (n_valid == 0) continue;
          result.feasible_torso_samples++;

          // 앞쪽 K 개 (디퓨전 모델 품질 순) 만 평균
          //   → shoulder 를 "초기 인덱스(= 고품질) 후보" 에 맞추도록 편향
          //   dd2_list 의 순서 = world_poses 의 순서 = 디퓨전 품질 순서
          //   (c_reach > 0.3 으로 skip 된 극단 후보만 빠져있고 나머지는 원순서)
          size_t k_use = std::min(K_BEST, dd2_list.size());
          double sum_dd2_topk = 0.0;
          for (size_t kk = 0; kk < k_use; ++kk) sum_dd2_topk += dd2_list[kk];

          double avg_reach      = sum_reach   / static_cast<double>(n_valid);
          double avg_dist       = sum_dist    / static_cast<double>(n_valid);
          double avg_dist_sweet = sum_dd2_topk / static_cast<double>(k_use);

          double dt3 = t3 - cur_t3, dt4 = t4 - cur_t4, dt5 = t5 - cur_t5;
          double c_torso_move = dt3 * dt3 + dt4 * dt4 + dt5 * dt5;

          double cost = w1_reach      * avg_reach
                      + w2_dist_sweet * avg_dist_sweet
                      + w3_torso      * c_torso_move;

          TorsoCandidate cand;
          cand.angles_deg = {t3 * 180.0 / M_PI, t4 * 180.0 / M_PI, t5 * 180.0 / M_PI};
          cand.cost = cost;
          cand.cost_reach = avg_reach;
          cand.cost_dist_sweet = avg_dist_sweet;   // (dist-D_IDEAL)^2 평균
          cand.cost_torso_move = c_torso_move;
          cand.avg_dist = avg_dist;                // 진단용 (실제 거리 평균)
          cand.shoulder_world = t_shoulder;
          all_cands.push_back(cand);
        }
      }
    }

    auto t_end = std::chrono::steady_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    if (result.feasible_torso_samples == 0) {
      RCLCPP_WARN(get_logger(), "[Phase0] No feasible torso samples found for %zu candidates", world_poses.size());
      result.success = false;
      return result;
    }

    result.success = true;
    
    std::sort(all_cands.begin(), all_cands.end(), [](const TorsoCandidate& a, const TorsoCandidate& b) {
      return a.cost < b.cost;
    });

    const size_t max_top = 10;
    for (size_t i = 0; i < std::min(max_top, all_cands.size()); ++i) {
      result.top_candidates.push_back(all_cands[i]);
    }

    auto best = result.top_candidates[0];

    result.best_torso_angles_deg = best.angles_deg;
    result.best_cost = best.cost;
    result.cost_reach = best.cost_reach;
    result.cost_manip_inv = 0.0; // 사용 안 함
    result.cost_dist_sweet = best.cost_dist_sweet;
    result.cost_torso_move = best.cost_torso_move;
    result.best_avg_dist = best.avg_dist;
    result.best_shoulder_x = best.shoulder_world.x();
    result.best_shoulder_y = best.shoulder_world.y();
    result.best_shoulder_z = best.shoulder_world.z();

    RCLCPP_INFO(get_logger(),
      "[Phase0] ✅ Best torso [t3,t4,t5] = [%.1f°, %.1f°, %.1f°]  cost=%.4f\n"
      "         breakdown: reach=%.4f  dist_sweet=%.4f (top-%zu mean of (d-D)^2)  torso_move=%.4f  (D_IDEAL=%.2fm)\n"
      "         best_avg_dist=%.3f m  shoulder_world=[%.3f, %.3f, %.3f]\n"
      "         feasible=%zu/%zu  %.1fms (stored top %zu cands)",
      result.best_torso_angles_deg[0], result.best_torso_angles_deg[1], result.best_torso_angles_deg[2],
      result.best_cost, result.cost_reach, result.cost_dist_sweet, K_BEST, result.cost_torso_move, D_IDEAL,
      result.best_avg_dist,
      result.best_shoulder_x, result.best_shoulder_y, result.best_shoulder_z,
      result.feasible_torso_samples, result.total_torso_samples, result.elapsed_ms, result.top_candidates.size());

    return result;
  }

  // ==========================================
  // Phase 1: Torso 이동 (Joint-space Planning)
  //
  // Phase 0에서 얻은 best_torso_angles_rad로 torso_3/4/5를 이동.
  // arm 조인트는 현재 자세 유지.
  // ==========================================
  Phase1Log runPhase1TorsoMove(
      moveit::planning_interface::MoveGroupInterface* mg_torso_arm,
      const std::vector<double>& best_torso_angles_rad)
  {
    auto t_start = std::chrono::steady_clock::now();
    Phase1Log result;
    result.attempted = true;
    result.target_torso_angles_deg = {
      best_torso_angles_rad[0] * 180.0 / M_PI,
      best_torso_angles_rad[1] * 180.0 / M_PI,
      best_torso_angles_rad[2] * 180.0 / M_PI
    };

    mg_torso_arm->setStartStateToCurrentState();
    auto cur_state = mg_torso_arm->getCurrentState();
    cur_state->update();

    const auto* jmg = cur_state->getJointModelGroup(mg_torso_arm->getName());
    if (!jmg) {
      result.failure_reason = "JMG not found: " + mg_torso_arm->getName();
      auto t_end = std::chrono::steady_clock::now();
      result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
      return result;
    }

    // 타겟: torso_3/4/5만 업데이트, 나머지는 현재 유지
    std::map<std::string, double> targets;
    for (const auto& jn : jmg->getVariableNames()) {
      targets[jn] = cur_state->getVariablePosition(jn);
    }
    targets["torso_3"] = best_torso_angles_rad[0];
    targets["torso_4"] = best_torso_angles_rad[1];
    targets["torso_5"] = best_torso_angles_rad[2];

    mg_torso_arm->setJointValueTarget(targets);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_res = mg_torso_arm->plan(plan);
    if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
      result.failure_reason = "Plan failed: " + errorCodeToString(plan_res.val);
      RCLCPP_WARN(get_logger(), "[Phase1] ⚠️ %s", result.failure_reason.c_str());
      auto t_end = std::chrono::steady_clock::now();
      result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
      return result;
    }
    result.trajectory_points = plan.trajectory_.joint_trajectory.points.size();

    // Torso 이동 속도를 대폭 줄임 (예: 20% 속도로 스케일링)
    scaleTrajectorySpeed(plan.trajectory_, 0.2);

    RCLCPP_INFO(get_logger(), "[Phase1] 🚀 Executing torso move (%zu points, speed scaled to 20%%)...",
                result.trajectory_points);
    auto exec_res = mg_torso_arm->execute(plan);
    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS) {
      result.failure_reason = "Execute failed: " + errorCodeToString(exec_res.val);
      RCLCPP_WARN(get_logger(), "[Phase1] ⚠️ %s", result.failure_reason.c_str());
      auto t_end = std::chrono::steady_clock::now();
      result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
      return result;
    }

    // 상태 안정화 대기
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    result.success = true;
    auto t_end = std::chrono::steady_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    RCLCPP_INFO(get_logger(), "[Phase1] ✅ Done. target [%.1f°, %.1f°, %.1f°]  %.1fms",
                result.target_torso_angles_deg[0], result.target_torso_angles_deg[1],
                result.target_torso_angles_deg[2], result.elapsed_ms);
    return result;
  }

  // ==========================================
  // 잡은 객체를 EE에 Attached Collision Object로 부착
  //
  // 캐시된 object point cloud 로부터 EE 프레임 기준 AABB를 계산하여
  // 적절한 크기의 Box primitive를 EE 링크에 attach 한다.
  // MoveIt이 이후 경로 계획 시 이 박스를 로봇 몸체 일부로 취급하여
  // 충돌을 회피한다.
  // ==========================================
  bool attachGraspedObject(
      const std::string& ee_link,
      const std::vector<std::string>& touch_links,
      moveit::planning_interface::MoveGroupInterface* active_group)
  {
    // 1. 캐시된 포인트 클라우드 가져오기
    sensor_msgs::msg::PointCloud2::SharedPtr pcd;
    {
      std::lock_guard<std::mutex> lock(object_pcd_mutex_);
      pcd = cached_object_pcd_;
    }

    if (!pcd || pcd->width * pcd->height == 0) {
      RCLCPP_WARN(get_logger(), "[Attach] No object point cloud cached. Skipping attach.");
      return false;
    }

    // 2. 현재 로봇 상태에서 EE 변환 가져오기
    auto current_state = active_group->getCurrentState();
    current_state->update();
    const Eigen::Isometry3d& T_ee = current_state->getGlobalLinkTransform(ee_link);
    Eigen::Isometry3d T_ee_inv = T_ee.inverse();

    // PC frame → planning frame 변환
    std::string pc_frame = pcd->header.frame_id;
    std::string planning_frame = active_group->getPlanningFrame();
    Eigen::Isometry3d T_pc_to_planning = Eigen::Isometry3d::Identity();
    if (!pc_frame.empty() && pc_frame != planning_frame) {
      if (current_state->knowsFrameTransform(pc_frame)) {
        // URDF 내 프레임 (link_torso_2 등)
        T_pc_to_planning = current_state->getFrameTransform(pc_frame);
        RCLCPP_INFO(get_logger(), "[Attach] Frame '%s' → '%s' via robot state.",
                    pc_frame.c_str(), planning_frame.c_str());
      } else if (tf_buffer_) {
        // URDF 외부 프레임 (world, odom 등) → TF2로 조회
        try {
          auto tf_stamped = tf_buffer_->lookupTransform(
              planning_frame, pc_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
          auto& t = tf_stamped.transform;
          T_pc_to_planning = Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
                           * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
          RCLCPP_INFO(get_logger(), "[Attach] Frame '%s' → '%s' via TF2: t=[%.3f, %.3f, %.3f]",
                      pc_frame.c_str(), planning_frame.c_str(),
                      t.translation.x, t.translation.y, t.translation.z);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_ERROR(get_logger(), "[Attach] TF2 lookup '%s'→'%s' failed: %s. Aborting attach.",
                       pc_frame.c_str(), planning_frame.c_str(), ex.what());
          return false;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "[Attach] No TF2 buffer and unknown frame '%s'. Aborting attach.",
                     pc_frame.c_str());
        return false;
      }
    }

    // 3. 포인트 클라우드 파싱 & EE 프레임에서 AABB 계산
    Eigen::Vector3d aabb_min(1e9, 1e9, 1e9);
    Eigen::Vector3d aabb_max(-1e9, -1e9, -1e9);
    size_t n_points = 0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcd, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pcd, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pcd, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x, y = *iter_y, z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      // PC frame → planning frame → EE frame
      Eigen::Vector3d p_planning = T_pc_to_planning * Eigen::Vector3d(x, y, z);
      Eigen::Vector3d p_ee = T_ee_inv * p_planning;

      aabb_min = aabb_min.cwiseMin(p_ee);
      aabb_max = aabb_max.cwiseMax(p_ee);
      n_points++;
    }

    if (n_points < 10) {
      RCLCPP_WARN(get_logger(), "[Attach] Too few valid points (%zu). Skipping attach.", n_points);
      return false;
    }

    // 4. Box 크기 및 중심 (EE 프레임 기준)
    const double MARGIN = 0.02;  // 2cm 안전 마진
    Eigen::Vector3d box_dims = (aabb_max - aabb_min) + Eigen::Vector3d::Constant(MARGIN);
    Eigen::Vector3d box_center = (aabb_max + aabb_min) / 2.0;

    // 최소 크기 보장 (너무 얇은 객체 방지)
    for (int d = 0; d < 3; ++d) {
      if (box_dims[d] < 0.03) box_dims[d] = 0.03;
    }

    RCLCPP_INFO(get_logger(),
      "[Attach] Object bbox in EE frame: dims=[%.3f, %.3f, %.3f] m, "
      "center=[%.3f, %.3f, %.3f], %zu points",
      box_dims.x(), box_dims.y(), box_dims.z(),
      box_center.x(), box_center.y(), box_center.z(), n_points);

    // 5. AttachedCollisionObject 생성
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = ee_link;
    aco.object.header.frame_id = ee_link;
    aco.object.id = "grasped_object";
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box_dims.x(), box_dims.y(), box_dims.z()};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = box_center.x();
    box_pose.position.y = box_center.y();
    box_pose.position.z = box_center.z();
    box_pose.orientation.w = 1.0;

    aco.object.primitives.push_back(primitive);
    aco.object.primitive_poses.push_back(box_pose);

    // touch_links: 이 링크들과는 항상 충돌 무시 (그리퍼 자체와 겹치므로)
    aco.touch_links = touch_links;

    // 6. Planning scene에 적용
    auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_req->scene.robot_state.attached_collision_objects.push_back(aco);
    apply_req->scene.is_diff = true;

    if (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "[Attach] ApplyPlanningScene service not available");
      return false;
    }

    auto result = apply_planning_scene_client_->async_send_request(apply_req);
    if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[Attach] Failed to attach object (timeout)");
      return false;
    }

    if (result.get()->success) {
      attached_object_id_ = "grasped_object";
      RCLCPP_INFO(get_logger(),
        "✅ [Attach] Attached collision box to '%s': "
        "[%.3f x %.3f x %.3f] m at EE-local (%.3f, %.3f, %.3f)",
        ee_link.c_str(), box_dims.x(), box_dims.y(), box_dims.z(),
        box_center.x(), box_center.y(), box_center.z());
      return true;
    }

    RCLCPP_ERROR(get_logger(), "[Attach] Failed to apply attached collision object");
    return false;
  }

  // ==========================================
  // EE에서 Attached Collision Object 분리
  //
  // 객체를 놓거나, 새로운 grasp 요청이 올 때 호출.
  // attach 된 객체를 planning scene 에서 제거한다.
  // ==========================================
  bool detachGraspedObject(const std::string& ee_link)
  {
    if (attached_object_id_.empty()) return true;

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = attached_object_id_;
    aco.link_name = ee_link;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    // world에서도 제거 (detach 후 scene에 남지 않도록)
    moveit_msgs::msg::CollisionObject remove_world;
    remove_world.id = attached_object_id_;
    remove_world.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_req->scene.robot_state.attached_collision_objects.push_back(aco);
    apply_req->scene.world.collision_objects.push_back(remove_world);
    apply_req->scene.is_diff = true;

    if (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "[Detach] ApplyPlanningScene service not available");
      return false;
    }

    auto result = apply_planning_scene_client_->async_send_request(apply_req);
    if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[Detach] Failed to detach object (timeout)");
      return false;
    }

    if (result.get()->success) {
      RCLCPP_INFO(get_logger(), "✅ [Detach] Removed '%s' from '%s'",
                  attached_object_id_.c_str(), ee_link.c_str());
      attached_object_id_.clear();
      return true;
    }

    RCLCPP_ERROR(get_logger(), "[Detach] Failed to remove collision object");
    return false;
  }

  // 저장된 궤적 초기화
  void clearStoredTrajectory()
  {
    stored_step1_trajectory_.joint_trajectory.points.clear();
    stored_step1_trajectory_.joint_trajectory.joint_names.clear();
    stored_step2_trajectory_.joint_trajectory.points.clear();
    stored_step2_trajectory_.joint_trajectory.joint_names.clear();
    stored_arm_id_.clear();
    trajectory_available_ = false;
  }

  // ==========================================
  // MoveIt 에러 코드 → 문자열
  // ==========================================
  std::string errorCodeToString(int error_code)
  {
    switch (error_code) {
      case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
      case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
      case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
      case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
      case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
      case moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
      case moveit_msgs::msg::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
      case moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";
      default:
        return "ERROR (code: " + std::to_string(error_code) + ")";
    }
  }

  // ==========================================
  // OctoMap 충돌 허용/비허용
  // ==========================================
  bool setOctomapCollisionAllowed(const std::vector<std::string>& link_names, bool allow)
  {
    try {
      auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      get_request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

      if (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "GetPlanningScene service not available");
        return false;
      }

      auto get_result = get_planning_scene_client_->async_send_request(get_request);
      if (get_result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Failed to get planning scene (timeout)");
        return false;
      }

      auto get_response = get_result.get();
      auto acm = get_response->scene.allowed_collision_matrix;

      const std::string octomap_id = "<octomap>";

      int octomap_idx = -1;
      for (size_t i = 0; i < acm.entry_names.size(); ++i) {
        if (acm.entry_names[i] == octomap_id) {
          octomap_idx = static_cast<int>(i);
          break;
        }
      }

      if (octomap_idx < 0) {
        acm.entry_names.push_back(octomap_id);
        octomap_idx = static_cast<int>(acm.entry_names.size() - 1);
        for (auto& entry : acm.entry_values) {
          entry.enabled.push_back(false);
        }
        moveit_msgs::msg::AllowedCollisionEntry octomap_entry;
        octomap_entry.enabled.resize(acm.entry_names.size(), false);
        acm.entry_values.push_back(octomap_entry);
      }

      for (const auto& link_name : link_names) {
        int link_idx = -1;
        for (size_t i = 0; i < acm.entry_names.size(); ++i) {
          if (acm.entry_names[i] == link_name) {
            link_idx = static_cast<int>(i);
            break;
          }
        }
        if (link_idx >= 0 && octomap_idx >= 0) {
          if (static_cast<size_t>(octomap_idx) < acm.entry_values[link_idx].enabled.size()) {
            acm.entry_values[link_idx].enabled[octomap_idx] = allow;
          }
          if (static_cast<size_t>(link_idx) < acm.entry_values[octomap_idx].enabled.size()) {
            acm.entry_values[octomap_idx].enabled[link_idx] = allow;
          }
        }
      }

      auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
      apply_request->scene.allowed_collision_matrix = acm;
      apply_request->scene.is_diff = true;

      if (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "ApplyPlanningScene service not available");
        return false;
      }

      auto apply_result = apply_planning_scene_client_->async_send_request(apply_request);
      if (apply_result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Failed to apply planning scene (timeout)");
        return false;
      }

      auto apply_response = apply_result.get();
      if (apply_response->success) {
        RCLCPP_INFO(get_logger(), "ACM updated: OctoMap collision %s for %zu links",
          allow ? "ALLOWED" : "RESTORED", link_names.size());
        return true;
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to apply ACM changes");
        return false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in setOctomapCollisionAllowed: %s", e.what());
      return false;
    }
  }

  // ==========================================
  // Trajectory를 최대 거리까지만 자르기
  // ==========================================
  moveit_msgs::msg::RobotTrajectory trimTrajectoryByDistance(
    const moveit_msgs::msg::RobotTrajectory& original_trajectory,
    moveit::core::RobotStatePtr robot_state,
    const moveit::core::JointModelGroup* jmg,
    const std::string& ee_link,
    double max_distance,
    double min_distance,
    double& actual_distance,
    bool& is_valid)
  {
    moveit_msgs::msg::RobotTrajectory trimmed;
    trimmed.joint_trajectory.joint_names = original_trajectory.joint_trajectory.joint_names;

    actual_distance = 0.0;
    is_valid = false;

    if (original_trajectory.joint_trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "[Trim] Empty trajectory received");
      return trimmed;
    }

    // 첫 번째 point의 EE 위치
    const auto& first_point = original_trajectory.joint_trajectory.points.front();
    robot_state->setJointGroupPositions(jmg, first_point.positions);
    robot_state->update();
    Eigen::Vector3d start_pos = robot_state->getGlobalLinkTransform(ee_link).translation();

    RCLCPP_INFO(get_logger(), "[Trim] Start EEF: [%.4f, %.4f, %.4f], limits: [%.4f, %.4f] m",
                start_pos.x(), start_pos.y(), start_pos.z(), min_distance, max_distance);

    for (size_t i = 0; i < original_trajectory.joint_trajectory.points.size(); ++i) {
      const auto& point = original_trajectory.joint_trajectory.points[i];

      robot_state->setJointGroupPositions(jmg, point.positions);
      robot_state->update();
      Eigen::Vector3d current_pos = robot_state->getGlobalLinkTransform(ee_link).translation();

      double dist = (current_pos - start_pos).norm();

      if (dist > max_distance) {
        RCLCPP_INFO(get_logger(), "[Trim] Point %zu exceeds max (%.4f > %.4f). Trimmed.", i, dist, max_distance);
        break;
      }

      trimmed.joint_trajectory.points.push_back(point);
      actual_distance = dist;
    }

    if (actual_distance < min_distance) {
      RCLCPP_WARN(get_logger(), "[Trim] Too short: %.4f < %.4f", actual_distance, min_distance);
    } else if (trimmed.joint_trajectory.points.size() < 2) {
      RCLCPP_WARN(get_logger(), "[Trim] Not enough points: %zu", trimmed.joint_trajectory.points.size());
    } else {
      RCLCPP_INFO(get_logger(), "[Trim] Valid: %zu points, %.4f m", trimmed.joint_trajectory.points.size(), actual_distance);
      is_valid = true;
    }

    return trimmed;
  }

  // ==========================================
  // Trajectory 속도 스케일링
  // ==========================================
  void scaleTrajectorySpeed(moveit_msgs::msg::RobotTrajectory& trajectory, double speed_scale)
  {
    if (speed_scale <= 0.0 || trajectory.joint_trajectory.points.empty()) return;

    double time_scale = 1.0 / speed_scale;

    for (auto& point : trajectory.joint_trajectory.points) {
      // 전체 시간을 nanosec 단위로 변환 후 스케일링
      double total_ns = static_cast<double>(point.time_from_start.sec) * 1e9
                      + static_cast<double>(point.time_from_start.nanosec);
      total_ns *= time_scale;

      point.time_from_start.sec = static_cast<int32_t>(total_ns / 1e9);
      point.time_from_start.nanosec = static_cast<uint32_t>(
        static_cast<uint64_t>(total_ns) % 1000000000ULL);

      for (auto& vel : point.velocities) vel *= speed_scale;
      for (auto& acc : point.accelerations) acc *= (speed_scale * speed_scale);
    }
  }

  // ==========================================
  // [개선3] Trajectory 역방향 생성
  // ==========================================
  moveit_msgs::msg::RobotTrajectory reverseTrajectory(
    const moveit_msgs::msg::RobotTrajectory& forward_traj)
  {
    moveit_msgs::msg::RobotTrajectory reversed;
    reversed.joint_trajectory.joint_names = forward_traj.joint_trajectory.joint_names;

    const auto& points = forward_traj.joint_trajectory.points;
    if (points.empty()) return reversed;

    // 전체 소요 시간 (nanosec 단위)
    const auto& last = points.back().time_from_start;
    double total_ns = static_cast<double>(last.sec) * 1e9
                    + static_cast<double>(last.nanosec);

    for (int i = static_cast<int>(points.size()) - 1; i >= 0; --i) {
      trajectory_msgs::msg::JointTrajectoryPoint pt = points[i];

      // 시간 역전: new_time = total_time - original_time
      double orig_ns = static_cast<double>(pt.time_from_start.sec) * 1e9
                     + static_cast<double>(pt.time_from_start.nanosec);
      double new_ns = total_ns - orig_ns;

      pt.time_from_start.sec = static_cast<int32_t>(new_ns / 1e9);
      pt.time_from_start.nanosec = static_cast<uint32_t>(
        static_cast<uint64_t>(new_ns) % 1000000000ULL);

      // 속도 부호 반전
      for (auto& vel : pt.velocities) vel = -vel;

      reversed.joint_trajectory.points.push_back(pt);
    }
    return reversed;
  }

  // ==========================================
  // [개선1] EEF 로컬 Z축 기준 Cartesian 접근 waypoint 생성
  //   - robot_state: step1 종료 상태 (FK로 EEF pose 추출)
  //   - ee_link: 끝단 링크명
  //   - approach_distance: 접근 거리 (EEF Z축 방향으로 이동)
  // ==========================================
  std::vector<geometry_msgs::msg::Pose> computeEefApproachWaypoints(
    const moveit::core::RobotStatePtr& robot_state,
    const std::string& ee_link,
    double approach_distance)
  {
    // step1 종료 상태에서의 EEF pose (planning frame 기준)
    const Eigen::Isometry3d& eef_tf = robot_state->getGlobalLinkTransform(ee_link);
    Eigen::Vector3d eef_pos = eef_tf.translation();
    Eigen::Matrix3d eef_rot = eef_tf.rotation();

    // EEF 로컬 Z축 방향 (planning frame으로 변환된 상태)
    Eigen::Vector3d local_z = eef_rot.col(2);

    RCLCPP_INFO(get_logger(), "[EEF Approach] EEF pos: [%.4f, %.4f, %.4f]",
                eef_pos.x(), eef_pos.y(), eef_pos.z());
    RCLCPP_INFO(get_logger(), "[EEF Approach] EEF local Z (in planning frame): [%.4f, %.4f, %.4f]",
                local_z.x(), local_z.y(), local_z.z());

    // EEF의 로컬 -Z 방향 (EEF가 바라보는 반대 방향)으로 approach
    Eigen::Vector3d target_pos = eef_pos - local_z * approach_distance;

    geometry_msgs::msg::Pose approach_pose;
    approach_pose.position.x = target_pos.x();
    approach_pose.position.y = target_pos.y();
    approach_pose.position.z = target_pos.z();

    // 자세(orientation)는 EEF 현재 자세 유지
    Eigen::Quaterniond q(eef_rot);
    approach_pose.orientation.x = q.x();
    approach_pose.orientation.y = q.y();
    approach_pose.orientation.z = q.z();
    approach_pose.orientation.w = q.w();

    RCLCPP_INFO(get_logger(), "[EEF Approach] Target pos (EEF -Z, %.3fm): [%.4f, %.4f, %.4f]",
                approach_distance, target_pos.x(), target_pos.y(), target_pos.z());

    return {approach_pose};
  }

  // ==========================================
  // step1 종료 상태를 RobotState로 구성
  // ==========================================
  moveit::core::RobotStatePtr buildStateFromTrajectoryEnd(
    moveit::planning_interface::MoveGroupInterface* move_group,
    const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    auto robot_state = move_group->getCurrentState();
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    const auto& last_point = trajectory.joint_trajectory.points.back();

    for (size_t j = 0; j < joint_names.size(); ++j) {
      robot_state->setJointPositions(joint_names[j], &last_point.positions[j]);
    }
    robot_state->update();
    return robot_state;
  }

  // ==========================================
  // [개선3] 플레이백 서비스 핸들러
  //   물체를 잡은 상태에서 base frame Z축 방향으로 7cm 상승
  //   → grasp pose publish
  // ==========================================
  void handle_playback(
    const std::shared_ptr<PlaybackGrasp::Request> request,
    std::shared_ptr<PlaybackGrasp::Response> response)
  {
    RCLCPP_INFO(get_logger(), "[Lift] Request received for arm: %s", request->arm_id.c_str());

    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (!trajectory_available_) {
      response->success = false;
      response->message = "No stored trajectory available.";
      RCLCPP_WARN(get_logger(), "[Lift] No trajectory stored.");
      return;
    }

    if (request->arm_id != stored_arm_id_) {
      response->success = false;
      response->message = "Stored trajectory is for arm '" + stored_arm_id_
                        + "', but requested '" + request->arm_id + "'.";
      RCLCPP_WARN(get_logger(), "[Lift] Arm mismatch.");
      return;
    }

    // 사용할 MoveGroup 선택
    moveit::planning_interface::MoveGroupInterface* active_group = nullptr;
    moveit::planning_interface::MoveGroupInterface* active_group_torso = nullptr;
    std::string target_ee_link;
    std::vector<std::string> gripper_links;

    if (request->arm_id == "right") {
      active_group = move_group_right_.get();
      active_group_torso = move_group_right_torso_.get();
      target_ee_link = "ee_right";
      gripper_links = {"ee_right", "ee_finger_r1", "ee_finger_r2",
                       "link_right_arm_5", "link_right_arm_6"};
    } else if (request->arm_id == "left") {
      active_group = move_group_left_.get();
      active_group_torso = move_group_left_torso_.get();
      target_ee_link = "ee_left";
      gripper_links = {"ee_left", "ee_finger_l1", "ee_finger_l2",
                       "link_left_arm_5", "link_left_arm_6"};
    }

    if (!active_group || !active_group_torso) {
      response->success = false;
      response->message = "MoveGroup not available for arm: " + request->arm_id;
      return;
    }

    // Base frame Z축 방향으로 7cm 상승
    const double LIFT_HEIGHT = 0.10;  // 7cm
    const int N_LIFT_WAYPOINTS = 20;   // 명시적 중간 경유지 개수 (직선성 보장)

    RCLCPP_INFO(get_logger(), "⬆️ [Lift] Lifting EEF +%.0fcm in base Z (with %d intermediate waypoints)...",
                LIFT_HEIGHT * 100, N_LIFT_WAYPOINTS);

    // 현재 EEF pose 가져오기
    auto current_state = active_group_torso->getCurrentState();
    current_state->update();
    const Eigen::Isometry3d& eef_tf = current_state->getGlobalLinkTransform(target_ee_link);

    Eigen::Vector3d start_pos = eef_tf.translation();
    Eigen::Quaterniond eef_q(eef_tf.rotation());
    Eigen::Vector3d target_pos = start_pos + Eigen::Vector3d(0, 0, LIFT_HEIGHT);

    // ==========================================
    // 명시적 중간 경유지: start → (N_LIFT_WAYPOINTS 등간격) → target
    //
    // 경유지를 1개(end-only)만 주면 MoveIt 가 full Cartesian 경로의
    // 각 샘플에서 IK 를 푸는데, 중간에 여러 IK basin 을 오가면서 단조증가
    // 아닌 "스윙" 궤적이 나올 수 있다. 등간격 waypoint 를 박으면
    // MoveIt 가 각 세그먼트를 독립적으로 선형 보간하므로 월드 +Z 직선성이
    // 더 강하게 보장된다. (orientation 은 모든 경유지에서 고정)
    // ==========================================
    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    lift_waypoints.reserve(N_LIFT_WAYPOINTS);
    for (int k = 1; k <= N_LIFT_WAYPOINTS; ++k) {
      double t = static_cast<double>(k) / N_LIFT_WAYPOINTS;
      Eigen::Vector3d p = start_pos + Eigen::Vector3d(0, 0, LIFT_HEIGHT * t);
      geometry_msgs::msg::Pose wp;
      wp.position.x = p.x();
      wp.position.y = p.y();
      wp.position.z = p.z();
      wp.orientation.x = eef_q.x();
      wp.orientation.y = eef_q.y();
      wp.orientation.z = eef_q.z();
      wp.orientation.w = eef_q.w();
      lift_waypoints.push_back(wp);
    }

    RCLCPP_INFO(get_logger(), "[Lift] Current EEF: [%.4f, %.4f, %.4f] → Target: [%.4f, %.4f, %.4f]",
                start_pos.x(), start_pos.y(), start_pos.z(),
                target_pos.x(), target_pos.y(), target_pos.z());

    // OctoMap 충돌 허용 (물체를 들고 있으므로)
    // attached collision object ('grasped_object')도 octomap과 충돌 허용해야
    // 물체를 들고 이동할 수 있음
    auto acm_links = gripper_links;
    if (!attached_object_id_.empty()) {
      acm_links.push_back(attached_object_id_);
      RCLCPP_INFO(get_logger(), "[Lift] Including attached object '%s' in OctoMap ACM allow list.",
                  attached_object_id_.c_str());
    }
    bool acm_modified = setOctomapCollisionAllowed(acm_links, true);

    // Cartesian path로 상승 (다중 waypoint)
    // ⚠️ setEndEffectorLink 를 명시적으로 호출해야 computeCartesianPath 가
    //    올바른 EE 링크(ee_right/ee_left)를 기준으로 Cartesian 경로를 생성한다.
    //    미지정 시 rby1_*_arm_torso 그룹의 기본 EE 가 사용되어 의도치 않은
    //    링크가 +Z 직선을 따르고, 실제 EE 는 비직선 궤적을 그린다.
    active_group_torso->setEndEffectorLink(target_ee_link);
    active_group_torso->setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory lift_trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    double fraction = active_group_torso->computeCartesianPath(
        lift_waypoints, 0.001, 0.0, lift_trajectory, true, &error_code);

    RCLCPP_INFO(get_logger(), "[Lift] Cartesian path: %.1f%% coverage, %zu points",
                fraction * 100.0, lift_trajectory.joint_trajectory.points.size());

    // ==========================================
    // [Lift 직선성 진단] 궤적의 각 점에서 EE FK 를 돌려
    // 월드 +Z 단조증가 및 X/Y 이탈을 검증한다.
    // ==========================================
    if (!lift_trajectory.joint_trajectory.points.empty()) {
      auto diag_state = std::make_shared<moveit::core::RobotState>(*current_state);
      const auto* jmg_torso = diag_state->getJointModelGroup(active_group_torso->getName());
      const auto& joint_names = lift_trajectory.joint_trajectory.joint_names;

      double max_xy_dev   = 0.0;
      double max_xy_dev_k = 0;
      double prev_z       = start_pos.z() - 1.0;  // 초기값 (항상 통과)
      bool   z_monotonic  = true;
      int    first_z_violation_k = -1;
      Eigen::Vector3d last_ee = start_pos;

      const auto& points = lift_trajectory.joint_trajectory.points;
      for (size_t k = 0; k < points.size(); ++k) {
        const auto& pt = points[k];
        for (size_t ji = 0; ji < joint_names.size() && ji < pt.positions.size(); ++ji) {
          diag_state->setJointPositions(joint_names[ji], &pt.positions[ji]);
        }
        diag_state->update();
        Eigen::Vector3d ee_k = diag_state->getGlobalLinkTransform(target_ee_link).translation();
        last_ee = ee_k;

        double dx = ee_k.x() - start_pos.x();
        double dy = ee_k.y() - start_pos.y();
        double xy_dev = std::sqrt(dx * dx + dy * dy);
        if (xy_dev > max_xy_dev) {
          max_xy_dev = xy_dev;
          max_xy_dev_k = k;
        }

        if (ee_k.z() + 1e-6 < prev_z) {
          z_monotonic = false;
          if (first_z_violation_k < 0) first_z_violation_k = static_cast<int>(k);
        }
        prev_z = ee_k.z();

        // 처음 3개 + 마지막 3개 점을 상세 로그
        if (k < 3 || k + 3 >= points.size()) {
          RCLCPP_INFO(get_logger(),
            "  [Lift traj %zu/%zu] EE=[%.4f, %.4f, %.4f]  dz=%+.4f  xy_dev=%.4f m",
            k + 1, points.size(), ee_k.x(), ee_k.y(), ee_k.z(),
            ee_k.z() - start_pos.z(), xy_dev);
        }
      }

      double total_dz = last_ee.z() - start_pos.z();
      RCLCPP_INFO(get_logger(),
        "[Lift Diag] total_dz=%+.4f m (expected %+.4f)  max_xy_dev=%.4f m @ pt %zu  z_monotonic=%s",
        total_dz, LIFT_HEIGHT, max_xy_dev, max_xy_dev_k,
        z_monotonic ? "YES" : "NO");

      if (!z_monotonic) {
        RCLCPP_WARN(get_logger(),
          "[Lift Diag] ⚠️ Z is NOT monotonically increasing! First violation at pt %d.",
          first_z_violation_k);
      }
      if (max_xy_dev > 0.01) {  // 1 cm 이탈 이상이면 경고
        RCLCPP_WARN(get_logger(),
          "[Lift Diag] ⚠️ XY deviation %.4f m > 1 cm — trajectory is NOT straight +Z.",
          max_xy_dev);
      }
      if (std::abs(total_dz - LIFT_HEIGHT) > 0.005) {  // 5 mm 오차 이상
        RCLCPP_WARN(get_logger(),
          "[Lift Diag] ⚠️ Final dz=%+.4f m differs from expected %+.4f m by > 5 mm.",
          total_dz, LIFT_HEIGHT);
      }
    }

    if (fraction < 0.8 || lift_trajectory.joint_trajectory.points.size() < 2) {
      RCLCPP_ERROR(get_logger(), "[Lift] Cartesian path insufficient (%.1f%%). Aborting.", fraction * 100.0);
      if (acm_modified) setOctomapCollisionAllowed(acm_links, false);
      response->success = false;
      response->message = "Lift Cartesian path failed: " + std::to_string(fraction * 100.0) + "% coverage";
      return;
    }

    // 속도 스케일링 후 실행 (실제 실행 생략 - Planning only)
    scaleTrajectorySpeed(lift_trajectory, 0.2);

    active_group_torso->setStartStateToCurrentState();
    auto lift_result = active_group_torso->execute(lift_trajectory);
    //moveit::core::MoveItErrorCode lift_result;
    //lift_result.val = moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(get_logger(), "[Lift] Cartesian 궤적 실제 실행 완료.");

    if (acm_modified) setOctomapCollisionAllowed(acm_links, false);

    if (lift_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "Lift execution failed: " + errorCodeToString(lift_result.val);
      RCLCPP_ERROR(get_logger(), "[Lift] Execution failed: %s", errorCodeToString(lift_result.val).c_str());
      return;
    }

    // 실제 도착 EEF 위치
    {
      geometry_msgs::msg::PoseStamped actual = active_group->getCurrentPose(target_ee_link);
      RCLCPP_INFO(get_logger(), "📍 [Lift Actual EEF] [%.4f, %.4f, %.4f]",
                  actual.pose.position.x, actual.pose.position.y, actual.pose.position.z);
    }

    RCLCPP_INFO(get_logger(), "[Lift] Done. EEF lifted +%.0fcm.", LIFT_HEIGHT * 100);

    // Grasp pose를 PoseArray에 저장하고 publish
    {
      if (stored_arm_id_ == "left") {
        grasp_poses_msg_.poses[0] = stored_grasp_pose_;
        left_grasp_valid_ = true;
        RCLCPP_INFO(get_logger(), "📦 [Grasp Pose] Left grasp pose updated.");
      } else if (stored_arm_id_ == "right") {
        grasp_poses_msg_.poses[1] = stored_grasp_pose_;
        right_grasp_valid_ = true;
        RCLCPP_INFO(get_logger(), "📦 [Grasp Pose] Right grasp pose updated.");
      }

      grasp_poses_msg_.header.stamp = this->now();
      grasp_pose_pub_->publish(grasp_poses_msg_);
      RCLCPP_INFO(get_logger(), "📡 [Grasp Pose] Published (left_valid=%s, right_valid=%s)",
                  left_grasp_valid_ ? "true" : "false",
                  right_grasp_valid_ ? "true" : "false");
    }

    clearStoredTrajectory();
    response->success = true;
    response->message = "Lift completed (+7cm Z). Grasp pose published.";
    RCLCPP_INFO(get_logger(), "[Lift] Complete.");
  }

  // ==========================================
  // [JointMove] 관절 목표값 기반 collision-aware planning & execution
  //
  // 플래닝 그룹의 active joint 순서대로 목표 각도(rad)를 받아
  // MoveIt 플래너가 충돌 회피 경로를 계획하고 실행한다.
  // ==========================================
  void handle_joint_move(
    const std::shared_ptr<JointMoveSrv::Request> request,
    std::shared_ptr<JointMoveSrv::Response> response)
  {
    RCLCPP_INFO(get_logger(), "📥 [JointMove] arm_id=%s, group=%s, %zu joint values, speed=%.2f",
        request->arm_id.c_str(), request->group_name.c_str(),
        request->joint_values.size(), request->speed_scale);

    // 1. MoveGroup 선택
    moveit::planning_interface::MoveGroupInterface* active_group = nullptr;
    std::string resolved_group;

    if (!request->group_name.empty()) {
      // 사용자가 직접 지정한 그룹
      resolved_group = request->group_name;
      if (resolved_group == "rby1_right_arm") active_group = move_group_right_.get();
      else if (resolved_group == "rby1_left_arm") active_group = move_group_left_.get();
      else if (resolved_group == "rby1_right_arm_torso") active_group = move_group_right_torso_.get();
      else if (resolved_group == "rby1_left_arm_torso") active_group = move_group_left_torso_.get();
    } else {
      // arm_id 기반 자동 선택 (torso 포함)
      if (request->arm_id == "right") {
        active_group = move_group_right_torso_.get();
        resolved_group = "rby1_right_arm_torso";
      } else if (request->arm_id == "left") {
        active_group = move_group_left_torso_.get();
        resolved_group = "rby1_left_arm_torso";
      }
    }

    if (!active_group) {
      response->success = false;
      response->message = "Unknown group or arm_id: group='" + request->group_name
                        + "', arm_id='" + request->arm_id + "'";
      RCLCPP_ERROR(get_logger(), "❌ [JointMove] %s", response->message.c_str());
      return;
    }

    // 2. Joint 이름 및 개수 확인
    auto current_state = active_group->getCurrentState();
    if (!current_state) {
      response->success = false;
      response->message = "Failed to get current robot state.";
      RCLCPP_ERROR(get_logger(), "❌ [JointMove] %s", response->message.c_str());
      return;
    }
    const auto* jmg = current_state->getJointModelGroup(resolved_group);
    if (!jmg) {
      response->success = false;
      response->message = "JointModelGroup '" + resolved_group + "' not found.";
      RCLCPP_ERROR(get_logger(), "❌ [JointMove] %s", response->message.c_str());
      return;
    }

    const auto& active_joints = jmg->getActiveJointModels();
    size_t n_joints = active_joints.size();

    if (request->joint_values.size() != n_joints) {
      response->success = false;
      response->message = "Joint count mismatch: got " + std::to_string(request->joint_values.size())
                        + ", expected " + std::to_string(n_joints) + " for '" + resolved_group + "'";
      RCLCPP_ERROR(get_logger(), "❌ [JointMove] %s", response->message.c_str());

      // 디버그용: 기대하는 관절 이름 출력
      std::string names_str;
      for (size_t i = 0; i < n_joints; ++i) {
        if (i > 0) names_str += ", ";
        names_str += active_joints[i]->getName();
      }
      response->message += ". Expected joints: [" + names_str + "]";
      RCLCPP_INFO(get_logger(), "[JointMove] Expected joints: [%s]", names_str.c_str());
      return;
    }

    // 응답에 사용된 관절 이름 기록
    response->joint_names.resize(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
      response->joint_names[i] = active_joints[i]->getName();
    }

    // 3. 목표 설정 및 로그
    std::string target_str;
    for (size_t i = 0; i < n_joints; ++i) {
      if (i > 0) target_str += ", ";
      target_str += active_joints[i]->getName() + "="
                  + std::to_string(request->joint_values[i] * 180.0 / M_PI) + "°";
    }
    RCLCPP_INFO(get_logger(), "[JointMove] Target: [%s]", target_str.c_str());

    active_group->setStartStateToCurrentState();
    active_group->setJointValueTarget(request->joint_values);

    // 4. Planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = active_group->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "[JointMove] Planning failed: " + errorCodeToString(plan_result.val);
      RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "✅ [JointMove] Plan succeeded (%zu points)",
                plan.trajectory_.joint_trajectory.points.size());

    // 5. 속도 스케일링
    double speed = request->speed_scale;
    if (speed <= 0.0 || speed > 1.0) speed = 0.3;
    scaleTrajectorySpeed(plan.trajectory_, speed);

    // 6. Execution
    RCLCPP_INFO(get_logger(), "🚀 [JointMove] Executing (speed=%.0f%%)...", speed * 100.0);
    active_group->setStartStateToCurrentState();
    auto exec_result = active_group->execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "[JointMove] Execution failed: " + errorCodeToString(exec_result.val);
      RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
      return;
    }

    response->success = true;
    response->message = "[JointMove] '" + resolved_group + "' moved successfully.";
    RCLCPP_INFO(get_logger(), "✅ [JointMove] Complete. Group='%s'", resolved_group.c_str());
  }

  // ==========================================
  // [개선2] 핵심 서비스 핸들러
  //   모든 후보에 대해 step1+step2를 먼저 계획,
  //   둘 다 성공한 경우에만 실행
  // ==========================================
  void handle_service(
    const std::shared_ptr<ExecuteGraspSrv::Request> request,
    std::shared_ptr<ExecuteGraspSrv::Response> response)
  {
    // mode 파싱: 빈 문자열 → "grasp" (하위 호환성)
    std::string mode = request->mode;
    if (mode.empty()) mode = "grasp";
    const bool is_place_mode = (mode == "place");

    RCLCPP_INFO(get_logger(), "📥Service Request: Arm [%s], Candidates [%zu], Mode [%s]",
        request->arm_id.c_str(), request->grasp_candidates.poses.size(), mode.c_str());

    // ==========================================
    // 디버그 로그 초기화
    // ==========================================
    GraspLog grasp_log;
    grasp_log.timestamp = currentTimestamp();
    grasp_log.arm_id = request->arm_id;
    grasp_log.total_candidates = request->grasp_candidates.poses.size();
    auto log_start = std::chrono::steady_clock::now();

    // 새 요청이 오면 이전 궤적 무효화 + 이전 attached object 제거
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (trajectory_available_) {
        RCLCPP_WARN(get_logger(), "[Trajectory] Clearing previous stored trajectory (arm: %s) - new request received.",
                    stored_arm_id_.c_str());
      }
      // 이전 grasp의 attached collision object 제거
      if (!attached_object_id_.empty()) {
        std::string prev_ee = (stored_arm_id_ == "left") ? "ee_left" : "ee_right";
        detachGraspedObject(prev_ee);
      }
      clearStoredTrajectory();
    }

    // ==========================================
    // 1. 사용할 팔 선택
    // ==========================================
    moveit::planning_interface::MoveGroupInterface* active_group = nullptr;
    moveit::planning_interface::MoveGroupInterface* active_group_torso = nullptr;
    std::string target_ee_link;
    std::string arm_base_link;  // Phase 0: 어깨 링크 (torso_5 아래의 arm base)
    std::vector<std::string> gripper_links;

    if (request->arm_id == "left") {
        active_group = move_group_left_.get();
        active_group_torso = move_group_left_torso_.get();
        target_ee_link = "ee_left";
        arm_base_link = "link_left_arm_0";
        gripper_links = {"ee_left", "ee_finger_l1", "ee_finger_l2",
                         "link_left_arm_5", "link_left_arm_6"};
    } else if (request->arm_id == "right") {
        active_group = move_group_right_.get();
        active_group_torso = move_group_right_torso_.get();
        target_ee_link = "ee_right";
        arm_base_link = "link_right_arm_0";
        gripper_links = {"ee_right", "ee_finger_r1", "ee_finger_r2",
                         "link_right_arm_5", "link_right_arm_6"};
    } else {
        response->success = false;
        response->message = "Invalid arm_id. Use 'left' or 'right'.";
        return;
    }

    if (!active_group || !active_group_torso) {
        response->success = false;
        response->message = "MoveGroup not initialized.";
        return;
    }

    // ==========================================
    // 2. Planning 준비
    // ==========================================
    std::string input_frame = request->grasp_candidates.header.frame_id;
    if (input_frame.empty()) input_frame = "link_torso_5";

    std::string planning_frame = active_group->getPlanningFrame();

    RCLCPP_INFO(get_logger(), "Input frame: [%s], Planning frame: [%s]",
                input_frame.c_str(), planning_frame.c_str());

    active_group->setPoseReferenceFrame(planning_frame);

    auto& poses = request->grasp_candidates.poses;
    size_t total = poses.size();

    // ==========================================
    // Pregrasp standoff & approach 거리 설계
    //
    // GraspGen 이 내려주는 grasp pose 는 "물체 접촉 지점" 자체이므로
    // Step 1 이 이 pose 에 그대로 계획하면 EE 가 octomap 에 박힌
    // 상태에서 종료되어 IK/Step 1 이 환경 충돌로 전부 거부된다.
    //
    // 해결: grasp pose 에서 EE 로컬 +Z (=-approach) 방향으로
    // PREGRASP_STANDOFF 만큼 뒤로 물린 "pregrasp pose" 를 Step 1 의
    // 목표로 쓰고, Step 2 Cartesian 이 그 거리만큼 -Z 로 밀어 넣어
    // 실제 grasp pose 에 도달하게 한다.
    //
    //   [Step 1]                        [Step 2]
    //   pregrasp (world_poses[i])  →   grasp (GraspGen 원본)
    //   = grasp + R_eef * (+Z * STANDOFF)
    //
    // Step 2 의 MIN/MAX/LONG 은 이 STANDOFF 를 커버해야 한다.
    // ==========================================
    // [Standoff 설계]
    //   pregrasp 은 EE 를 grasp 에서 접근 방향 반대(local +Z)로 뒤로 빼어
    //   Step 1 목표가 물체 octomap 내부에 놓이지 않도록 한다.
    //
    //   이전 테스트에서 8 cm standoff + 7-DOF IK 일 때 arm_5/finger 가 테이블
    //   octomap 과 충돌 → collision 전량 거부. 이 문제는 standoff 크기가 아닌
    //   arm 자세의 문제이며, 10-DOF IK(torso+arm) 전환으로 해결한다.
    //   (torso 각도를 함께 풀어 arm 이 테이블을 피하는 자세를 찾음)
    //
    //   ⚠️ LONG 이 수십 cm 이상이면 Cartesian IK discontinuity → trim fail.
    // Place 모드에서는 standoff/approach 불필요 (목표 pose에 직접 계획)
    const double PREGRASP_STANDOFF      = is_place_mode ? 0.0 : 0.08;   // place: 0, grasp: 8cm
    const double MIN_APPROACH_DISTANCE  = 0.095;   // Step 2 trim 하한 (grasp only)
    const double MAX_APPROACH_DISTANCE  = 0.11;   // Step 2 trim 상한 (grasp only)
    const double LONG_APPROACH_DISTANCE = 0.12;   // Step 2 plan sweep 길이 (grasp only)
    const double STEP1_SPEED_SCALE      = is_place_mode ? 0.2 : 0.3;    // place: 느리게
    const double CARTESIAN_SPEED_SCALE  = 0.2;    // Step 2 (approach) 속도 스케일

    if (is_place_mode) {
      RCLCPP_INFO(get_logger(),
        "[PLACE MODE] Processing %zu candidates (no standoff, no Step2 cartesian)...", total);
    } else {
      RCLCPP_INFO(get_logger(),
        "Processing %zu candidates (pregrasp standoff=%.0fcm, approach trim=%.0f~%.1fcm)...",
        total, PREGRASP_STANDOFF * 100,
        MIN_APPROACH_DISTANCE * 100, MAX_APPROACH_DISTANCE * 100);
    }

    // ==========================================
    // 2-0. Grasp 후보를 planning frame(world)로 사전 변환
    //      Phase 0/1 및 IK 루프에서 재사용
    //      주의: Phase 1 torso 이동 전에 현재 torso 상태로 변환해야 한다.
    //      (input_frame이 link_torso_5인 경우, torso 이동 후 변환하면
    //       world 좌표가 함께 움직여 target이 왜곡된다)
    //
    //      [Antipodal yaw flip — 제거됨]
    //        과거 본 노드 안에서 수행했던 카메라 자세 회피용 yaw flip 은
    //        grasp_prefilter_node 로 분리되었다. prefilter 가
    //        (원본, flip) 두 자세를 모두 후보로 보내고 카메라 자세 점수
    //        순으로 정렬해 두므로, 본 노드는 들어온 후보를 그대로 처리한다.
    //
    //      [Rigid-block pre-filter — 제거됨]
    //        그리퍼/카메라 콜리전 사전 검사도 prefilter 로 이동.
    //        본 노드는 도착 후보가 모두 rigid-block 통과 상태임을 가정한다.
    // ==========================================
    std::vector<geometry_msgs::msg::Pose> world_poses;
    world_poses.reserve(total);
    {
      auto pre_state = active_group->getCurrentState();
      pre_state->update();
      Eigen::Isometry3d frame_tf = Eigen::Isometry3d::Identity();
      bool need_transform = (input_frame != planning_frame);
      if (need_transform) {
        if (pre_state->knowsFrameTransform(input_frame)) {
          frame_tf = pre_state->getFrameTransform(input_frame);
        } else {
          RCLCPP_WARN(get_logger(), "Frame [%s] unknown; assuming already in planning frame.",
                      input_frame.c_str());
          need_transform = false;
        }
      }
      for (const auto& p : poses) {
        Eigen::Isometry3d p_in;
        p_in.setIdentity();
        p_in.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        p_in.linear() = Eigen::Quaterniond(
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
        Eigen::Isometry3d p_w = need_transform ? (frame_tf * p_in) : p_in;

        // ---- Pregrasp standoff: grasp pose 에서 EE 로컬 +Z 로 뒤로 물림 ----
        // approach_dir = -local_z 이므로 +local_z 는 "후퇴" 방향.
        Eigen::Vector3d plus_z_world = p_w.rotation().col(2);
        p_w.translation() += plus_z_world * PREGRASP_STANDOFF;

        geometry_msgs::msg::Pose wp;
        wp.position.x = p_w.translation().x();
        wp.position.y = p_w.translation().y();
        wp.position.z = p_w.translation().z();
        Eigen::Quaterniond qw(p_w.rotation());
        wp.orientation.x = qw.x();
        wp.orientation.y = qw.y();
        wp.orientation.z = qw.z();
        wp.orientation.w = qw.w();
        world_poses.push_back(wp);
      }
      RCLCPP_INFO(get_logger(), "[Frame] Pre-computed %zu world poses (transform=%s).",
                  world_poses.size(), need_transform ? "yes" : "identity");
    }

    // ==========================================
    // Phase 0/Phase 1 제거 — 10-DOF 통합 계획으로 전환
    //
    // [이전 방식]
    //   Phase 0: torso 그리드 탐색 (거리 휴리스틱)
    //   Phase 1: torso 이동 (~6.5 s 실행)
    //   Step 1 : 7-DOF arm-only 계획
    //
    // [문제점]
    //   - Phase 0 거리 기반 cost 로는 arm 자세가 테이블과 충돌하는지 예측 불가
    //   - torso 를 고정한 뒤 7-DOF IK 를 풀면, 특정 torso 각도에서만
    //     충돌 없는 해가 존재하는 후보들이 전량 거부됨
    //   - 매 후보마다 다른 torso 가 최적일 수 있는데, 하나의 torso 를 공유
    //
    // [새 방식]
    //   Step 1 에서 rby1_*_arm_torso (10-DOF) 그룹으로 IK + 계획
    //   → TRAC-IK 가 torso+arm 을 동시에 풀어 충돌을 자연스럽게 회피
    //   → RRTConnect 등 10-DOF 플래너가 장애물을 피하는 경로 생성
    //   → Phase 0/Phase 1 불필요 (torso 는 Step 1 에서 arm 과 함께 이동)
    //
    //   함수 runPhase0TorsoOptimization / runPhase1TorsoMove 는 삭제하지 않고
    //   호출만 스킵하여, 필요 시 쉽게 복원 가능.
    // ==========================================
    RCLCPP_INFO(get_logger(),
      "========== Phase 0/1 SKIPPED: 10-DOF integrated planning mode ==========");



    // ==========================================
    // 2-1. Planning Scene 스냅샷 (충돌 검사용)
    //      Phase 1 + ACM 변경 이후에 캡처하여 새 torso 자세 + allow 상태 반영
    // ==========================================
    planning_scene::PlanningScenePtr current_scene;
    {
      auto current_state_tmp = active_group->getCurrentState();
      auto robot_model = current_state_tmp->getRobotModel();
      current_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

      auto ps_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      ps_req->components.components =
          moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
          moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
          moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
          moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
          moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
          moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;

      auto ps_future = get_planning_scene_client_->async_send_request(ps_req);
      if (ps_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        current_scene->usePlanningSceneMsg(ps_future.get()->scene);
        RCLCPP_INFO(get_logger(), "Planning scene loaded for collision-aware IK.");
      } else {
        RCLCPP_WARN(get_logger(), "Failed to get planning scene. IK will skip collision check.");
        current_scene = nullptr;
      }
    }

    // ==========================================
    // IK validity callback: collision + torso 변화량 제한 통합
    //
    // TRAC-IK 가 내부적으로 해를 찾을 때마다 이 callback 이 호출된다.
    // callback 이 false 를 반환하면 TRAC-IK 는 timeout 내에서 다른 해를 재시도.
    // → "구하고 나서 버리기" 대신 "구할 때부터 제약" 으로 전환
    // ==========================================
    const std::vector<std::string> torso_joint_names = {"torso_3", "torso_4", "torso_5"};
    const double TORSO_MAX_DELTA_RAD = 15.0 * M_PI / 180.0;  // ±15°

    // 현재 torso 값 캡처 (IK 전체 과정에서 공유)
    std::vector<double> current_torso_vals(torso_joint_names.size(), 0.0);
    {
      auto cur_st = active_group_torso->getCurrentState();
      for (size_t tj = 0; tj < torso_joint_names.size(); ++tj) {
        const auto* jm = cur_st->getJointModel(torso_joint_names[tj]);
        if (jm) current_torso_vals[tj] = cur_st->getJointPositions(jm)[0];
      }
      RCLCPP_INFO(get_logger(),
        "[IK] Current torso: [%.1f°, %.1f°, %.1f°], limit: ±%.0f°",
        current_torso_vals[0] * 180.0 / M_PI,
        current_torso_vals[1] * 180.0 / M_PI,
        current_torso_vals[2] * 180.0 / M_PI,
        TORSO_MAX_DELTA_RAD * 180.0 / M_PI);
    }

    // callback 내부 진단 카운터 (외부에서 읽기 위해 shared)
    // TRAC-IK 내부 retry 횟수를 포함하므로 N_IK_ATTEMPTS 보다 클 수 있음
    struct ValidityStats {
      std::atomic<size_t> n_torso_reject{0};
      std::atomic<size_t> n_collision_reject{0};
    };
    auto validity_stats = std::make_shared<ValidityStats>();

    moveit::core::GroupStateValidityCallbackFn validity_callback;
    if (current_scene) {
      validity_callback = [&current_scene, &torso_joint_names, &current_torso_vals,
                           TORSO_MAX_DELTA_RAD, validity_stats, this](
          moveit::core::RobotState* state,
          const moveit::core::JointModelGroup* group,
          const double* joint_values) -> bool {
        state->setJointGroupPositions(group, joint_values);
        state->update();

        // (1) Torso 변화량 제한
        for (size_t tj = 0; tj < torso_joint_names.size(); ++tj) {
          const auto* jm = state->getJointModel(torso_joint_names[tj]);
          if (!jm) continue;
          double delta = std::abs(state->getJointPositions(jm)[0] - current_torso_vals[tj]);
          if (delta > TORSO_MAX_DELTA_RAD) {
            validity_stats->n_torso_reject++;
            return false;
          }
        }

        // (2) 환경+자체 충돌 검사
        bool in_collision = current_scene->isStateColliding(*state, group->getName());
        if (in_collision) {
          validity_stats->n_collision_reject++;
          return false;
        }
        return true;
      };
    }

    // ==========================================
    // 영공간(Null-space) 사후 평가 함수
    //
    // IK는 cost 없이 순수하게 풀어 EEF 정밀도를 최대화하고,
    // 여러 IK 해 중에서 아래 기준으로 최적의 해를 사후 선택:
    //
    // [A] 방향성 조작성 (Directional Manipulability)
    //     w_d² = u^T · J_trans · J_trans^T · u
    //     u = 목표 pose의 로컬 -Z (접근 방향)
    //
    // [B] 조인트 한계 회피 (Joint Centering)
    //     Σ ((θ_i - θ_mid) / (θ_max - θ_min))²
    // ==========================================
    const auto compute_nullspace_cost = [this, &target_ee_link, &torso_joint_names, &current_torso_vals](
        const moveit::core::RobotState& state,
        const moveit::core::JointModelGroup* jmg,
        const Eigen::Vector3d& approach_dir,
        double& out_manip, double& out_joint) -> double {

      auto& mut_state = const_cast<moveit::core::RobotState&>(state);
      mut_state.update();

      const moveit::core::LinkModel* ee_link_model = state.getLinkModel(target_ee_link);
      Eigen::MatrixXd jacobian;
      Eigen::Vector3d ref_point = Eigen::Vector3d::Zero();

      out_manip = 0.0;
      out_joint = 0.0;

      if (!state.getJacobian(jmg, ee_link_model, ref_point, jacobian, false)) {
        return 0.0;
      }

      // [A] 방향성 조작성
      Eigen::MatrixXd J_trans = jacobian.topRows(3);
      double w_d_sq = approach_dir.transpose() * (J_trans * J_trans.transpose()) * approach_dir;
      out_manip = w_d_sq;

      // [B] 조인트 한계 회피
      const auto& bounds = jmg->getActiveJointModelsBounds();
      const auto& joint_models = jmg->getActiveJointModels();

      for (size_t i = 0; i < joint_models.size(); ++i) {
        if (bounds[i]->empty()) continue;
        const auto& b = (*bounds[i])[0];
        if (!b.position_bounded_) continue;

        double pos = state.getJointPositions(joint_models[i])[0];
        double mid = (b.max_position_ + b.min_position_) / 2.0;
        double range = b.max_position_ - b.min_position_;
        if (range > 1e-6) {
          double normalized = (pos - mid) / range;
          out_joint += normalized * normalized;
        }
      }

      // [C] Torso 변화량 페널티: 현재 torso 에서 적게 움직이는 해 선호
      //     Σ (Δθ_torso / 15°)²  — 15° = TORSO_MAX_DELTA_RAD 기준
      double torso_penalty = 0.0;
      for (size_t tj = 0; tj < torso_joint_names.size(); ++tj) {
        const auto* jm = state.getJointModel(torso_joint_names[tj]);
        if (!jm) continue;
        double delta = state.getJointPositions(jm)[0] - current_torso_vals[tj];
        double normalized = delta / (15.0 * M_PI / 180.0);  // 15° 기준 정규화
        torso_penalty += normalized * normalized;
      }

      // cost = -5·manipulability + 1·joint_centering + 3·torso_penalty
      // torso_penalty 가중치 3.0: torso 를 조금씩만 쓰도록 유도
      return -5.0 * w_d_sq + 1.0 * out_joint + 3.0 * torso_penalty;
    };

    // ==========================================
    // 3. 배치 처리: BATCH_SIZE개씩 IK(다중 시도) → 사후 평가 → planning
    //
    // [10-DOF IK NOTE]
    //   rby1_*_arm_torso JMG (10-DOF) 로 IK 를 풀어 torso+arm 을
    //   동시에 최적화한다. TRAC-IK 가 torso 각도도 함께 탐색하므로:
    //     - 테이블 등 환경과 충돌하지 않는 arm 자세를 자연스럽게 발견
    //     - Phase 0/Phase 1 의 거리 기반 heuristic 이 불필요
    //     - 10-DOF 는 탐색 공간이 넓어 timeout 을 0.8s 로 상향
    //
    //   전략:
    //     - N_IK_ATTEMPTS = 6 (10-DOF 는 해가 더 풍부, 시도 수 축소)
    //     - IK_TIMEOUT = 0.8s (10-DOF TRAC-IK 내부 restart 에 충분)
    //     - 첫 2회는 현재자세 seed
    //     - 나머지 4회는 전체 10-DOF randomize
    //     - 조기 중단: 3회 연속 실패 시 abort (10-DOF 에 여유 부여)
    // ==========================================
    const size_t BATCH_SIZE        = 3;
    const size_t N_IK_ATTEMPTS     = 6;     // 후보당 IK 시도 (10-DOF 는 해가 풍부)
    const double IK_TIMEOUT        = 0.8;   // 시도당 timeout [s] (10-DOF 탐색 공간)
    const size_t N_DETAILED_CANDS  = 3;     // 배치 내 앞쪽 N개 후보만 per-attempt 상세 로그
    const size_t N_DETAILED_ATT    = 6;     // 후보당 상세 로그할 시도 횟수

    for (size_t batch_start = 0; batch_start < total; batch_start += BATCH_SIZE) {
      size_t batch_end = std::min(batch_start + BATCH_SIZE, total);

      RCLCPP_INFO(get_logger(), "==============================");
      RCLCPP_INFO(get_logger(), "[Batch] Processing candidates %zu~%zu / %zu",
                  batch_start + 1, batch_end, total);

      // 배치별 로그 엔트리 생성
      grasp_log.batches.emplace_back();
      BatchLog& batch_log = grasp_log.batches.back();
      batch_log.batch_index = batch_start / BATCH_SIZE;
      batch_log.candidate_range = std::to_string(batch_start + 1) + "~" + std::to_string(batch_end);

      // ------------------------------------------
      // [Phase 1] 배치 내 모든 후보에 대해 순수 IK × N회 → 사후 평가
      // ------------------------------------------
      struct IkCandidate {
        size_t original_index;
        moveit::core::RobotStatePtr state;
        geometry_msgs::msg::Pose pose_world;
        double ns_cost;
        double manip;
        double joint_center;
        double pos_err;
        double orient_err_deg;
      };
      std::vector<IkCandidate> ik_candidates;

      // 배치 내에서 공통으로 어깨 world 위치를 한 번만 측정 (현재 torso 상태 기준)
      // 10-DOF IK 에서는 torso 가 함께 움직이므로 IK 해 마다 shoulder 위치가 달라진다.
      // 여기서는 현재(=초기) shoulder 위치를 참고용으로만 기록한다.
      Eigen::Vector3d shoulder_world = Eigen::Vector3d::Zero();
      {
        auto cur_state_sh = active_group_torso->getCurrentState();
        cur_state_sh->update();
        const auto* arm_base_lm_ik = cur_state_sh->getLinkModel(arm_base_link);
        if (arm_base_lm_ik) {
          shoulder_world = cur_state_sh->getGlobalLinkTransform(arm_base_lm_ik).translation();
        }
        RCLCPP_INFO(get_logger(),
          "[IK Batch %zu] shoulder(%s) world=[%.3f, %.3f, %.3f]  "
          "JMG=%s (%zu DOF, torso+arm)  IK solver=TRAC-IK",
          batch_start / BATCH_SIZE, arm_base_link.c_str(),
          shoulder_world.x(), shoulder_world.y(), shoulder_world.z(),
          active_group_torso->getName().c_str(),
          active_group_torso->getCurrentState()->getJointModelGroup(
              active_group_torso->getName())->getActiveJointModels().size());
      }

      for (size_t i = batch_start; i < batch_end; ++i) {
        // 현재 state (IK seed) — 10-DOF (torso+arm) 그룹으로 IK 수행
        auto ref_state = active_group_torso->getCurrentState();
        const auto* jmg_ik = ref_state->getJointModelGroup(active_group_torso->getName());

        // Phase 0 이전에 사전 계산된 world frame pose를 재사용.
        // (torso 이동 후 link_torso_5 프레임 기준 변환을 다시 하면 target이 함께 움직이므로
        //  반드시 pre-computed world_poses를 사용해야 한다.)
        const geometry_msgs::msg::Pose& target_pose_world = world_poses[i];

        // 목표 접근 방향 (로컬 -Z)
        Eigen::Vector3d approach_dir;
        {
          Eigen::Quaterniond target_q(
              target_pose_world.orientation.w, target_pose_world.orientation.x,
              target_pose_world.orientation.y, target_pose_world.orientation.z);
          approach_dir = -target_q.toRotationMatrix().col(2);
        }

        // 어깨↔타겟 거리 (도달 가능성 사전 확인용)
        Eigen::Vector3d tgt_pos(target_pose_world.position.x,
                                 target_pose_world.position.y,
                                 target_pose_world.position.z);
        double dist_from_shoulder = (tgt_pos - shoulder_world).norm();

        RCLCPP_INFO(get_logger(),
          "[IK] Cand %zu: EE target [%.3f, %.3f, %.3f]  approach [%.2f, %.2f, %.2f]  "
          "dist_from_shoulder=%.3f m",
          i + 1,
          target_pose_world.position.x, target_pose_world.position.y, target_pose_world.position.z,
          approach_dir.x(), approach_dir.y(), approach_dir.z(),
          dist_from_shoulder);

        // ---- Rigid-block pre-filter (제거됨, prefilter 노드로 분리) ----
        // 본 노드 도착 시점에서 모든 후보는 grasp_prefilter_node 의
        // rigid-block + 카메라 콜리전 검사를 이미 통과했다고 가정한다.
        // 로그 호환을 위해 placeholder 엔트리만 기록.
        // (rigid_block_skip 은 항상 false, detail 도 빈 문자열)

        // 목표 pose 정보 (정밀도 검증용)
        Eigen::Quaterniond target_q_check(
            target_pose_world.orientation.w, target_pose_world.orientation.x,
            target_pose_world.orientation.y, target_pose_world.orientation.z);
        Eigen::Vector3d target_pos_check(
            target_pose_world.position.x, target_pose_world.position.y, target_pose_world.position.z);

        // IK를 N회 시도.
        // validity_callback 이 collision + torso 제한을 모두 처리하므로
        // setFromIK 가 true 를 반환하면 collision-free + torso ±15° 이내 보장.
        // TRAC-IK 내부에서 callback 거부 시 timeout 내 다른 해를 재시도한다.
        kinematics::KinematicsQueryOptions ik_opts;
        ik_opts.return_approximate_solution = true;

        moveit::core::RobotStatePtr best_state = nullptr;
        double best_ns_cost = std::numeric_limits<double>::max();
        double best_manip = 0, best_joint = 0;
        double best_pos_err = 0, best_orient_deg = 0;
        size_t n_precise = 0;

        // 진단 카운터
        size_t n_found        = 0;   // setFromIK 가 true 를 반환 (validity 통과 포함)
        double min_pos_err        = 1e9;
        double min_orient_err_deg = 1e9;

        // validity callback 카운터 리셋 (후보별 집계)
        validity_stats->n_torso_reject.store(0);
        validity_stats->n_collision_reject.store(0);

        const bool detailed_log =
            (i - batch_start) < N_DETAILED_CANDS;   // 앞쪽 후보만 상세 로그

        // [조기 종료] 연속 setFromIK=FALSE 카운터.
        //   10-DOF IK 는 탐색 공간이 넓어 current seed 2회가 실패해도
        //   random seed 에서 성공할 수 있다. 3회 연속 실패 (current 2 + random 1)
        //   이면 도달 불가로 판단하고 남은 시도를 절약한다.
        size_t consecutive_fail = 0;
        const size_t EARLY_ABORT_FAILS = 3;

        for (size_t attempt = 0; attempt < N_IK_ATTEMPTS; ++attempt) {
          auto try_state = std::make_shared<moveit::core::RobotState>(*ref_state);

          // Seed 전략:
          //  - attempt 0, 1 : 현재자세 seed (TRAC-IK 가 자연스러운 해 우선 시도)
          //  - attempt >= 2 : torso 는 현재값 ±15° 내 랜덤, arm 은 전체 랜덤
          if (attempt >= 2) {
            // arm 관절만 전체 범위 랜덤
            try_state->setToRandomPositions(jmg_ik);
            // torso 관절을 현재값 ±15° 범위 내로 재설정
            static thread_local std::mt19937 rng(std::random_device{}());
            for (size_t tj = 0; tj < torso_joint_names.size(); ++tj) {
              const auto* jm = try_state->getJointModel(torso_joint_names[tj]);
              if (!jm) continue;
              const auto& vb = jm->getVariableBounds(torso_joint_names[tj]);
              double lo = std::max(vb.min_position_, current_torso_vals[tj] - TORSO_MAX_DELTA_RAD);
              double hi = std::min(vb.max_position_, current_torso_vals[tj] + TORSO_MAX_DELTA_RAD);
              std::uniform_real_distribution<double> dist(lo, hi);
              double val = dist(rng);
              try_state->setJointPositions(jm, &val);
            }
          }

          // validity_callback: TRAC-IK 내부에서 해를 찾을 때마다 호출
          // → collision 또는 torso ±15° 초과 시 거부, timeout 내 재시도
          auto t_ik0 = std::chrono::steady_clock::now();
          bool found = try_state->setFromIK(
              jmg_ik, target_pose_world, target_ee_link,
              IK_TIMEOUT, validity_callback, ik_opts);
          double ik_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - t_ik0).count();

          if (!found) {
            if (detailed_log && attempt < N_DETAILED_ATT) {
              RCLCPP_INFO(get_logger(),
                "  [IK att %zu] Cand %zu: setFromIK=FALSE (%.1f ms, seed=%s)",
                attempt, i + 1, ik_ms, (attempt < 2 ? "current" : "random"));
            }
            consecutive_fail++;
            if (consecutive_fail >= EARLY_ABORT_FAILS && n_found == 0) {
              RCLCPP_WARN(get_logger(),
                "  [IK] Cand %zu: %zu consecutive setFromIK=FALSE (dist=%.3fm) "
                "— aborting remaining %zu attempts.",
                i + 1, consecutive_fail, dist_from_shoulder,
                N_IK_ATTEMPTS - attempt - 1);
              break;
            }
            continue;
          }
          consecutive_fail = 0;
          n_found++;

          // setFromIK + validity_callback 이 true 를 반환 →
          // collision-free + torso ±15° 이내 보장.
          // 정밀도 검증만 수행.
          try_state->update();
          const Eigen::Isometry3d& actual_eef = try_state->getGlobalLinkTransform(target_ee_link);

          Eigen::Quaterniond actual_q(actual_eef.rotation());
          double quat_dot = std::abs(actual_q.dot(target_q_check));
          double orient_err_rad = 2.0 * std::acos(std::min(quat_dot, 1.0));
          double pos_err = (actual_eef.translation() - target_pos_check).norm();
          double orient_err_deg = orient_err_rad * 180.0 / M_PI;

          if (pos_err < min_pos_err) min_pos_err = pos_err;
          if (orient_err_deg < min_orient_err_deg) min_orient_err_deg = orient_err_deg;

          bool precise = (pos_err <= 0.01 && orient_err_rad <= 0.17);
          if (detailed_log && attempt < N_DETAILED_ATT) {
            RCLCPP_INFO(get_logger(),
              "  [IK att %zu] Cand %zu: VALID (%.1f ms, seed=%s) "
              "pos_err=%.4f m  orient_err=%.2f°  precise=%s",
              attempt, i + 1, ik_ms, (attempt < 2 ? "current" : "random"),
              pos_err, orient_err_deg, precise ? "YES" : "NO");
          }
          if (!precise) continue;

          // Null-space 사후 평가
          n_precise++;

          double manip_val, joint_val;
          double ns_cost = compute_nullspace_cost(*try_state, jmg_ik, approach_dir, manip_val, joint_val);

          if (ns_cost < best_ns_cost) {
            best_ns_cost = ns_cost;
            best_state = try_state;
            best_manip = manip_val;
            best_joint = joint_val;
            best_pos_err = pos_err;
            best_orient_deg = orient_err_deg;
          }
        }

        // 루프 종료 후 요약 진단 로그
        // validity_callback 내 거부 횟수 (TRAC-IK 내부 retry 포함)
        size_t cb_torso = validity_stats->n_torso_reject.load();
        size_t cb_coll  = validity_stats->n_collision_reject.load();
        RCLCPP_INFO(get_logger(),
          "[IK Debug] Cand %zu: valid=%zu/%zu  precise=%zu  "
          "cb_reject(torso=%zu, collision=%zu)  min_pos=%.4f m  min_orient=%.1f°",
          i + 1, n_found, N_IK_ATTEMPTS, n_precise,
          cb_torso, cb_coll, min_pos_err, min_orient_err_deg);

        

        if (!best_state) {
          RCLCPP_INFO(get_logger(), "[IK] Candidate %zu: no precise solution in %zu attempts", i + 1, N_IK_ATTEMPTS);
          batch_log.ik_results.push_back({i + 1, false, "", 0, 0.0, 0.0, 0.0});
          continue;
        }

        RCLCPP_INFO(get_logger(), "[IK] Candidate %zu: ✅ %zu precise solutions, best: pos=%.4fm orient=%.1f° "
                    "manip=%.4f jnt=%.4f ns_cost=%.4f",
                    i + 1, n_precise, best_pos_err, best_orient_deg,
                    best_manip, best_joint, best_ns_cost);

        batch_log.ik_results.push_back({i + 1, false, "", n_precise, best_ns_cost, best_manip, best_joint});

        ik_candidates.push_back({i, best_state, target_pose_world, best_ns_cost,
                                 best_manip, best_joint, best_pos_err, best_orient_deg});
      }

      if (ik_candidates.empty()) {
        RCLCPP_WARN(get_logger(), "[Batch] No IK solutions in this batch. Next batch.");
        continue;
      }

      // ------------------------------------------
      // [Phase 2] cost 오름차순 정렬 (낮은 ns_cost = 조작성 높고 조인트 여유 큼)
      // ------------------------------------------
      std::sort(ik_candidates.begin(), ik_candidates.end(),
          [](const IkCandidate& a, const IkCandidate& b) {
            return a.ns_cost < b.ns_cost;
          });

      RCLCPP_INFO(get_logger(), "[Batch] %zu IK solutions found. Sorted by null-space cost:", ik_candidates.size());
      for (size_t r = 0; r < ik_candidates.size(); ++r) {
        RCLCPP_INFO(get_logger(), "  Rank %zu: candidate %zu (ns_cost=%.4f, manip=%.4f, jnt=%.4f)",
                    r + 1, ik_candidates[r].original_index + 1,
                    ik_candidates[r].ns_cost, ik_candidates[r].manip, ik_candidates[r].joint_center);
      }

      // ------------------------------------------
      // [Phase 3] 상위 점수(top-K) 후보만 Step1 + Step2 planning 시도
      // ------------------------------------------
      const size_t TOP_K_PLANNING = 3;
      const size_t n_plan_candidates = std::min(TOP_K_PLANNING, ik_candidates.size());
      RCLCPP_INFO(get_logger(), "[Batch] Planning only top %zu candidate(s) by null-space cost.", n_plan_candidates);

      for (size_t rank = 0; rank < n_plan_candidates; ++rank) {
        auto& cand = ik_candidates[rank];
        RCLCPP_INFO(get_logger(), "------------------------------");
        RCLCPP_INFO(get_logger(), "[Plan] Trying rank %zu / %zu -> candidate %zu (manip=%.4f, jnt=%.4f)",
                    rank + 1, n_plan_candidates, cand.original_index + 1, cand.manip, cand.joint_center);

        // Plan attempt 로그 엔트리 생성
        batch_log.plan_attempts.emplace_back();
        PlanAttemptLog& pa_log = batch_log.plan_attempts.back();
        pa_log.rank = rank + 1;
        pa_log.candidate_index = cand.original_index + 1;
        pa_log.step1_result = "PENDING";
        pa_log.step2_coverage = 0.0;
        pa_log.step2_trim_result = "PENDING";
        pa_log.trim_distance = 0.0;
        pa_log.direction_alignment = 0.0;
        pa_log.overall_result = "PENDING";
        pa_log.failure_reason = "";

        // Step 1: Joint-space planning (torso+arm, 10-DOF)
        // 10-DOF IK 해에는 torso 관절도 포함되어 있으므로, torso+arm 그룹으로
        // 계획하면 torso 이동과 arm 이동이 한 번에 처리된다.
        // Phase 0/Phase 1 으로 torso 를 미리 옮길 필요 없음.
        active_group_torso->setStartStateToCurrentState();
        active_group_torso->setJointValueTarget(*cand.state);

        moveit::planning_interface::MoveGroupInterface::Plan step1_plan;
        auto step1_result = active_group_torso->plan(step1_plan);

        if (step1_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "⚠️[Step1 Plan] FAILED: %s. Trying next ranked.",
                      errorCodeToString(step1_result.val).c_str());
          pa_log.step1_result = errorCodeToString(step1_result.val);
          pa_log.overall_result = "STEP1_PLAN_FAIL";
          pa_log.failure_reason = "Step1 planning failed: " + errorCodeToString(step1_result.val);
          continue;
        }
        RCLCPP_INFO(get_logger(), "[Step1 Plan] SUCCESS (%zu points)",
                    step1_plan.trajectory_.joint_trajectory.points.size());
        pa_log.step1_result = "SUCCESS";

        // ==========================================
        // [PLACE MODE] Step2 스킵 — 바로 Step1만 실행
        // ==========================================
        if (is_place_mode) {
          pa_log.step2_coverage = 100.0;
          pa_log.step2_trim_result = "SKIPPED_PLACE_MODE";

          scaleTrajectorySpeed(step1_plan.trajectory_, STEP1_SPEED_SCALE);
          RCLCPP_INFO(get_logger(), "🚀 [PLACE Step1 Exec] Moving to place pose (torso+arm, 10-DOF, speed=%.0f%%)...",
                      STEP1_SPEED_SCALE * 100.0);
          active_group_torso->setStartStateToCurrentState();
          auto exec1_result = active_group_torso->execute(step1_plan);

          if (exec1_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(get_logger(), "⚠️ [PLACE Step1 Exec] FAILED: %s. Trying next ranked.",
                        errorCodeToString(exec1_result.val).c_str());
            pa_log.overall_result = "STEP1_EXEC_FAIL";
            pa_log.failure_reason = "Place Step1 execution failed: " + errorCodeToString(exec1_result.val);
            continue;
          }
          rclcpp::sleep_for(std::chrono::milliseconds(500));

          // Place 도착 후 EEF 위치 로그
          {
            geometry_msgs::msg::PoseStamped final_pose = active_group->getCurrentPose(target_ee_link);
            RCLCPP_INFO(get_logger(), "📍 [PLACE Actual EEF] [%.4f, %.4f, %.4f]",
                        final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);
          }

          // Place 완료 후 attached object detach (물체를 놓음)
          if (!attached_object_id_.empty()) {
            RCLCPP_INFO(get_logger(), "🔓 [PLACE] Detaching object '%s' from '%s'",
                        attached_object_id_.c_str(), target_ee_link.c_str());
            detachGraspedObject(target_ee_link);
          }

          pa_log.overall_result = "SUCCESS";
          grasp_log.final_success = true;
          grasp_log.winning_candidate = cand.original_index + 1;
          grasp_log.winning_batch = batch_log.batch_index;
          grasp_log.winning_rank = rank + 1;
          {
            auto log_end = std::chrono::steady_clock::now();
            grasp_log.total_elapsed_ms =
              std::chrono::duration<double, std::milli>(log_end - log_start).count();
          }
          writeGraspLog(grasp_log);

          response->success = true;
          response->message = "[PLACE] Candidate " + std::to_string(cand.original_index + 1)
                            + " placed successfully.";
          RCLCPP_INFO(get_logger(), "✅ [PLACE] Complete for candidate %zu.", cand.original_index + 1);
          return;
        }

        // ==========================================
        // [GRASP MODE] 이하 기존 grasp 시퀀스 (Step 2 Cartesian 포함)
        // ==========================================

        // Step 2 직전에만 Gripper↔OctoMap ACM Allow를 적용 (사용자 요청 사항)
        // Step 1 및 IK 단계에서는 모든 충돌을 고려하도록 하여
        // 정말 안전한(물체와 전혀 겹치지 않는) 예비 위치에서만 잡기를 시도함
        bool gripper_acm_allowed = setOctomapCollisionAllowed(gripper_links, true);
        if (gripper_acm_allowed) {
          RCLCPP_INFO(get_logger(),
            "[ACM] gripper(%zu links) <-> <octomap> allow ENABLED for Step 2.",
            gripper_links.size());
        }

        auto acm_restore_fn = [this, gripper_links](void* p) {
          if (p) {
            this->setOctomapCollisionAllowed(gripper_links, false);
            RCLCPP_INFO(this->get_logger(),
              "[ACM] gripper <-> <octomap> allow RESTORED.");
          }
        };
        std::unique_ptr<void, decltype(acm_restore_fn)> acm_guard(
            gripper_acm_allowed ? reinterpret_cast<void*>(1) : nullptr,
            acm_restore_fn);

        // Step 2: Cartesian approach (torso+arm 그룹 사용)
        auto step1_end_state = buildStateFromTrajectoryEnd(active_group_torso, step1_plan.trajectory_);
        const auto* jmg = step1_end_state->getJointModelGroup(active_group_torso->getName());

        auto waypoints = computeEefApproachWaypoints(step1_end_state, target_ee_link, LONG_APPROACH_DISTANCE);
        active_group_torso->setEndEffectorLink(target_ee_link);  // 명시적 EE 링크 설정
        active_group_torso->setStartState(*step1_end_state);

        moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;

        double fraction = active_group_torso->computeCartesianPath(
          waypoints, 0.0005, 0.0, cartesian_trajectory, true, &error_code);

        RCLCPP_INFO(get_logger(), "[Step2 Plan] Cartesian (torso+arm, collision ON): %.1f%% coverage, %zu points",
                    fraction * 100.0, cartesian_trajectory.joint_trajectory.points.size());

        pa_log.step2_coverage = fraction * 100.0;

        double min_required_fraction = MIN_APPROACH_DISTANCE / LONG_APPROACH_DISTANCE;
        if (fraction < min_required_fraction) {
          RCLCPP_WARN(get_logger(), "[Step2 Plan] Cartesian coverage too low: %.1f%% (need %.1f%%). Trying next.",
                      fraction * 100.0, min_required_fraction * 100.0);
          pa_log.step2_trim_result = "LOW_COVERAGE";
          pa_log.overall_result = "STEP2_COVERAGE_FAIL";
          pa_log.failure_reason = "Cartesian coverage too low: "
                                + std::to_string(fraction * 100.0) + "%";
          continue;
        }

        if (cartesian_trajectory.joint_trajectory.points.size() < 2) {
          RCLCPP_WARN(get_logger(), "[Step2 Plan] Not enough points. Trying next ranked.");
          pa_log.step2_trim_result = "INSUFFICIENT_POINTS";
          pa_log.overall_result = "STEP2_POINTS_FAIL";
          pa_log.failure_reason = "Cartesian trajectory fewer than 2 points";
          continue;
        }

        double actual_distance = 0.0;
        bool trim_valid = false;
        auto trimmed_step2 = trimTrajectoryByDistance(
          cartesian_trajectory, step1_end_state, jmg, target_ee_link,
          MAX_APPROACH_DISTANCE, MIN_APPROACH_DISTANCE,
          actual_distance, trim_valid);

        if (!trim_valid) {
          RCLCPP_WARN(get_logger(), "[Step2 Plan] Trimmed trajectory invalid. Trying next ranked.");
          pa_log.step2_trim_result = "TRIM_INVALID";
          pa_log.trim_distance = actual_distance;
          pa_log.overall_result = "STEP2_TRIM_FAIL";
          pa_log.failure_reason = "Trimmed trajectory out of [min,max] distance range";
          continue;
        }

        // 이동 방향 검증: 실제 이동이 의도한 EEF -Z 방향과 일치하는지
        {
          const Eigen::Isometry3d& eef_tf_check = step1_end_state->getGlobalLinkTransform(target_ee_link);
          Eigen::Vector3d intended_dir = -eef_tf_check.rotation().col(2);  // EEF 로컬 -Z
          intended_dir.normalize();

          // 트림된 궤적의 첫 점과 마지막 점에서 EEF 위치 비교
          auto check_state = step1_end_state;
          check_state->setJointGroupPositions(jmg, trimmed_step2.joint_trajectory.points.front().positions);
          check_state->update();
          Eigen::Vector3d trim_start = check_state->getGlobalLinkTransform(target_ee_link).translation();

          check_state->setJointGroupPositions(jmg, trimmed_step2.joint_trajectory.points.back().positions);
          check_state->update();
          Eigen::Vector3d trim_end = check_state->getGlobalLinkTransform(target_ee_link).translation();

          Eigen::Vector3d actual_move = trim_end - trim_start;
          double actual_move_len = actual_move.norm();
          if (actual_move_len > 1e-6) {
            actual_move /= actual_move_len;
            double alignment = intended_dir.dot(actual_move);
            pa_log.direction_alignment = alignment;
            RCLCPP_INFO(get_logger(), "[Step2 Check] Direction alignment: %.3f (intended vs actual, 1.0=perfect)",
                        alignment);
            if (alignment < 0.5) {
              RCLCPP_WARN(get_logger(), "[Step2 Plan] Movement direction misaligned (%.3f < 0.5). "
                          "Robot would move sideways, not toward object. Trying next.", alignment);
              pa_log.step2_trim_result = "MISALIGNED";
              pa_log.trim_distance = actual_distance;
              pa_log.overall_result = "STEP2_ALIGN_FAIL";
              pa_log.failure_reason = "Movement misaligned with intended EEF -Z (alignment="
                                    + std::to_string(alignment) + ")";
              continue;
            }
          }
        }

        // Step2 plan 통과 — 트림 단계 성공
        pa_log.step2_trim_result = "SUCCESS";
        pa_log.trim_distance = actual_distance;

        scaleTrajectorySpeed(trimmed_step2, CARTESIAN_SPEED_SCALE);

        RCLCPP_INFO(get_logger(), "✅==> Step1 + Step2 planning SUCCEEDED for candidate %zu "
                    "(pos_err=%.4f, orient_err=%.1f°, manip=%.4f, jnt=%.4f)",
                    cand.original_index + 1, cand.pos_err, cand.orient_err_deg,
                    cand.manip, cand.joint_center);

        // ==========================================
        // 📋 디버그 요약: 선택된 후보의 계획 상세
        // ==========================================
        {
          // (1) 요청받은 원본 pose
          const auto& orig = poses[cand.original_index];
          RCLCPP_INFO(get_logger(), "📌 [Request Pose] input_frame=%s  pos=[%.4f, %.4f, %.4f]  quat=[%.4f, %.4f, %.4f, %.4f]",
                      input_frame.c_str(),
                      orig.position.x, orig.position.y, orig.position.z,
                      orig.orientation.x, orig.orientation.y, orig.orientation.z, orig.orientation.w);
          RCLCPP_INFO(get_logger(), "📌 [World Pose]   planning_frame=%s  pos=[%.4f, %.4f, %.4f]",
                      planning_frame.c_str(),
                      cand.pose_world.position.x, cand.pose_world.position.y, cand.pose_world.position.z);

          // (2) Null-space 최적화 결과
          RCLCPP_INFO(get_logger(), "📐 [Null-Space] directional_manipulability=%.4f  joint_centering=%.4f  total_cost=%.4f",
                      cand.manip, cand.joint_center, cand.ns_cost);

          // (3) EEF 접근 방향 (Step2에서 사용될 방향)
          Eigen::Vector3d approach_dir = -step1_end_state->getGlobalLinkTransform(target_ee_link).rotation().col(2);
          RCLCPP_INFO(get_logger(), "🎯 [Approach Dir] EEF local -Z = [%.3f, %.3f, %.3f]",
                      approach_dir.x(), approach_dir.y(), approach_dir.z());

          // (4) 각 조인트의 가동 범위 여유 (한계 도달 위험도)
          const auto* jmg_dbg = cand.state->getJointModelGroup(active_group_torso->getName());
          const auto& jm_dbg = jmg_dbg->getActiveJointModels();
          const auto& b_dbg = jmg_dbg->getActiveJointModelsBounds();
          std::string joint_margin_str;
          for (size_t ji = 0; ji < jm_dbg.size(); ++ji) {
            if (b_dbg[ji]->empty() || !(*b_dbg[ji])[0].position_bounded_) continue;
            double pos = cand.state->getJointPositions(jm_dbg[ji])[0];
            double lo = (*b_dbg[ji])[0].min_position_;
            double hi = (*b_dbg[ji])[0].max_position_;
            double margin_lo = pos - lo;
            double margin_hi = hi - pos;
            double min_margin = std::min(margin_lo, margin_hi);
            const std::string& jname = jm_dbg[ji]->getName();
            auto arm_pos = jname.find("arm_");
            std::string short_name = (arm_pos != std::string::npos) ? jname.substr(arm_pos) : jname;
            joint_margin_str += short_name
                             + ":" + std::to_string(static_cast<int>(min_margin * 180.0 / M_PI)) + "° ";
          }
          RCLCPP_INFO(get_logger(), "🔧 [Joint Margins] %s", joint_margin_str.c_str());

          // (5) Step1 계획 종료 EEF 위치
          Eigen::Vector3d step1_eef = step1_end_state->getGlobalLinkTransform(target_ee_link).translation();
          RCLCPP_INFO(get_logger(), "📍 [Step1 Plan End EEF] [%.4f, %.4f, %.4f]",
                      step1_eef.x(), step1_eef.y(), step1_eef.z());
        }

        // ------------------------------------------
        // 실제 실행 (비활성화: #if 0, 활성화: #if 1)
        // ------------------------------------------
#if 1
        // Step 1 실행 (torso+arm, 10-DOF)
        // 속도 스케일 적용 — torso+arm 동시 이동이므로 안전을 위해 감속
        scaleTrajectorySpeed(step1_plan.trajectory_, STEP1_SPEED_SCALE);
        RCLCPP_INFO(get_logger(), "🚀 [Step1 Exec] Moving to pre-grasp (torso+arm, 10-DOF, speed=%.0f%%)...",
                    STEP1_SPEED_SCALE * 100.0);
        active_group_torso->setStartStateToCurrentState();
        auto exec1_result = active_group_torso->execute(step1_plan);

        if (exec1_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "⚠️ [Step1 Exec] FAILED: %s. Trying next ranked.",
                      errorCodeToString(exec1_result.val).c_str());
          pa_log.overall_result = "STEP1_EXEC_FAIL";
          pa_log.failure_reason = "Step1 execution failed: " + errorCodeToString(exec1_result.val);
          continue;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // Step1 실제 도착 EEF
        {
          geometry_msgs::msg::PoseStamped step1_actual = active_group->getCurrentPose(target_ee_link);
          RCLCPP_INFO(get_logger(), "📍 [Step1 Actual EEF] [%.4f, %.4f, %.4f]",
                      step1_actual.pose.position.x, step1_actual.pose.position.y, step1_actual.pose.position.z);
        }

        // Step 2 실행 (torso+arm 그룹)
        RCLCPP_INFO(get_logger(), "🚀 [Step2 Exec] Cartesian approach (torso+arm, %.4f m, EEF local -Z)...", actual_distance);
        active_group_torso->setStartStateToCurrentState();
        auto exec2_result = active_group_torso->execute(trimmed_step2);

        if (exec2_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "⚠️ [Step2 Exec] FAILED: %s. Trying next ranked.",
                      errorCodeToString(exec2_result.val).c_str());
          pa_log.overall_result = "STEP2_EXEC_FAIL";
          pa_log.failure_reason = "Step2 execution failed: " + errorCodeToString(exec2_result.val);
          continue;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Step2 실제 도착 EEF + 이동량 → grasp pose 저장
        {
          geometry_msgs::msg::PoseStamped final_pose = active_group->getCurrentPose(target_ee_link);
          RCLCPP_INFO(get_logger(), "📍 [Step2 Actual EEF] [%.4f, %.4f, %.4f]",
                      final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);

          // Grasp pose 저장 (playback 완료 후 publish)
          std::lock_guard<std::mutex> gp_lock(trajectory_mutex_);
          stored_grasp_pose_ = final_pose.pose;
          RCLCPP_INFO(get_logger(), "📦 [Grasp Pose] Stored for arm '%s'", request->arm_id.c_str());
        }

        // 잡은 객체를 EE에 Attached Collision Object로 부착
        // → 이후 경로 계획(lift 등)에서 MoveIt이 로봇 몸체 일부로 인식
        attachGraspedObject(target_ee_link, gripper_links, active_group);
#endif

        RCLCPP_INFO(get_logger(), "[RViz Only] Planning succeeded. Skipping execution. "
                    "Check RViz 'Planned Path' display to visualize.");

        // ACM 복원은 acm_guard (unique_ptr 소멸자) 가 수행함.

        // 궤적 저장
        {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          stored_step1_trajectory_ = step1_plan.trajectory_;
          stored_step2_trajectory_ = trimmed_step2;
          stored_arm_id_ = request->arm_id;
          trajectory_available_ = true;
        }

        response->success = true;
        response->message = "Candidate " + std::to_string(cand.original_index)
                          + " planned (manip=" + std::to_string(cand.manip)
                          + ", jnt=" + std::to_string(cand.joint_center) + ").";
        RCLCPP_INFO(get_logger(), "==> Plan complete for candidate %zu. No execution.",
                    cand.original_index + 1);

        // 최종 성공 — 로그 마감 및 파일 기록
        pa_log.overall_result = "SUCCESS";
        grasp_log.final_success = true;
        grasp_log.winning_candidate = cand.original_index + 1;
        grasp_log.winning_batch = batch_log.batch_index;
        grasp_log.winning_rank = rank + 1;
        {
          auto log_end = std::chrono::steady_clock::now();
          grasp_log.total_elapsed_ms =
            std::chrono::duration<double, std::milli>(log_end - log_start).count();
        }
        writeGraspLog(grasp_log);
        return;

      } // end top-k candidate loop

      RCLCPP_WARN(get_logger(), "[Batch] Top-%zu candidates failed planning. Next batch.", n_plan_candidates);

    } // end batch loop

    response->success = false;
    response->message = "All candidates failed for arm: " + request->arm_id;
    RCLCPP_WARN(get_logger(), "All candidates exhausted.");

    // 최종 실패 — 로그 마감 및 파일 기록
    grasp_log.final_success = false;
    {
      auto log_end = std::chrono::steady_clock::now();
      grasp_log.total_elapsed_ms =
        std::chrono::duration<double, std::milli>(log_end - log_start).count();
    }
    writeGraspLog(grasp_log);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Rby1ActionServer>();
  node->init_moveit();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
