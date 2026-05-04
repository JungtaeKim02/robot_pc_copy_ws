#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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

// bio_ik는 kinematics.yaml에서 런타임 플러그인으로 로드됨
// PickNikRobotics/bio_ik (ros2 브랜치)는 표준 MoveIt API + cost function 사용
// #include <bio_ik/bio_ik.h> 는 불필요

#include "inha_interfaces/srv/execute_grasp_srv.hpp"
#include "inha_interfaces/srv/playback_grasp.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>
#include <vector>

using ExecuteGraspSrv = inha_interfaces::srv::ExecuteGraspSrv;
using PlaybackGrasp = inha_interfaces::srv::PlaybackGrasp;

class Rby1ActionServer : public rclcpp::Node
{
public:
  explicit Rby1ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rby1_grasp_action_server", options)
  {
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    service_server_ = this->create_service<ExecuteGraspSrv>(
      "execute_grasp_service",
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

    get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    apply_planning_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");

    RCLCPP_INFO(get_logger(), "Rby1 Grasp Action Server Created. Waiting for MoveIt...");
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

    if(move_group_right_) move_group_right_->startStateMonitor(2.0);
    if(move_group_left_) move_group_left_->startStateMonitor(2.0);

    while (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /get_planning_scene service...");
    }
    while (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for /apply_planning_scene service...");
    }

    RCLCPP_INFO(get_logger(), "MoveIt Interface initialized. Ready.");
  }

private:
  rclcpp::Service<ExecuteGraspSrv>::SharedPtr service_server_;
  rclcpp::Service<PlaybackGrasp>::SharedPtr playback_server_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;

  // 플레이백을 위한 궤적 저장
  std::mutex trajectory_mutex_;
  moveit_msgs::msg::RobotTrajectory stored_step1_trajectory_;
  moveit_msgs::msg::RobotTrajectory stored_step2_trajectory_;
  std::string stored_arm_id_;
  bool trajectory_available_ = false;

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
  //   step2 역방향 → step1 역방향 순서로 실행
  // ==========================================
  void handle_playback(
    const std::shared_ptr<PlaybackGrasp::Request> request,
    std::shared_ptr<PlaybackGrasp::Response> response)
  {
    RCLCPP_INFO(get_logger(), "[Playback] Request received for arm: %s", request->arm_id.c_str());

    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (!trajectory_available_) {
      response->success = false;
      response->message = "No stored trajectory available for playback.";
      RCLCPP_WARN(get_logger(), "[Playback] No trajectory stored.");
      return;
    }

    if (request->arm_id != stored_arm_id_) {
      response->success = false;
      response->message = "Stored trajectory is for arm '" + stored_arm_id_
                        + "', but requested '" + request->arm_id + "'.";
      RCLCPP_WARN(get_logger(), "[Playback] Arm mismatch.");
      return;
    }

    // 사용할 MoveGroup 선택
    moveit::planning_interface::MoveGroupInterface* active_group = nullptr;
    if (request->arm_id == "right") {
      active_group = move_group_right_.get();
    } else if (request->arm_id == "left") {
      active_group = move_group_left_.get();
    }

    if (!active_group) {
      response->success = false;
      response->message = "MoveGroup not available for arm: " + request->arm_id;
      return;
    }

    // Step 2 역방향 실행 (물체에서 후퇴)
    RCLCPP_INFO(get_logger(), "[Playback] Reversing Step 2 (retract)...");
    auto rev_step2 = reverseTrajectory(stored_step2_trajectory_);
    scaleTrajectorySpeed(rev_step2, 0.3);

    active_group->setStartStateToCurrentState();
    auto result2 = active_group->execute(rev_step2);
    if (result2 != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "Step 2 reverse execution failed: " + errorCodeToString(result2.val);
      RCLCPP_ERROR(get_logger(), "[Playback] Step 2 reverse failed: %s", errorCodeToString(result2.val).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "[Playback] Step 2 reverse done.");

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Step 1 역방향 실행 (원위치 복귀)
    RCLCPP_INFO(get_logger(), "[Playback] Reversing Step 1 (return to start)...");
    auto rev_step1 = reverseTrajectory(stored_step1_trajectory_);

    active_group->setStartStateToCurrentState();
    auto result1 = active_group->execute(rev_step1);
    if (result1 != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "Step 1 reverse execution failed: " + errorCodeToString(result1.val);
      RCLCPP_ERROR(get_logger(), "[Playback] Step 1 reverse failed: %s", errorCodeToString(result1.val).c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "[Playback] Step 1 reverse done. Returned to start.");

    clearStoredTrajectory();
    response->success = true;
    response->message = "Playback completed. Robot returned to start position.";
    RCLCPP_INFO(get_logger(), "[Playback] Complete.");
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
    RCLCPP_INFO(get_logger(), "Service Request: Arm [%s], Candidates [%zu]",
        request->arm_id.c_str(), request->grasp_candidates.poses.size());

    // 새 요청이 오면 이전 궤적 무효화
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (trajectory_available_) {
        RCLCPP_WARN(get_logger(), "[Trajectory] Clearing previous stored trajectory (arm: %s) - new request received.",
                    stored_arm_id_.c_str());
      }
      clearStoredTrajectory();
    }

    // ==========================================
    // 1. 사용할 팔 선택
    // ==========================================
    moveit::planning_interface::MoveGroupInterface* active_group = nullptr;
    std::string target_ee_link;
    std::vector<std::string> gripper_links;

    if (request->arm_id == "left") {
        active_group = move_group_left_.get();
        target_ee_link = "ee_left";
        gripper_links = {"ee_left", "ee_finger_l1", "ee_finger_l2",
                         "link_left_arm_5", "link_left_arm_6"};
    } else if (request->arm_id == "right") {
        active_group = move_group_right_.get();
        target_ee_link = "ee_right";
        gripper_links = {"ee_right", "ee_finger_r1", "ee_finger_r2",
                         "link_right_arm_5", "link_right_arm_6"};
    } else {
        response->success = false;
        response->message = "Invalid arm_id. Use 'left' or 'right'.";
        return;
    }

    if (!active_group) {
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

    const double MIN_APPROACH_DISTANCE = 0.035;
    const double MAX_APPROACH_DISTANCE = 0.05;
    const double LONG_APPROACH_DISTANCE = 0.15;  // 충분히 긴 거리 (잘라낼 용도)
    const double CARTESIAN_SPEED_SCALE = 0.2;

    RCLCPP_INFO(get_logger(), "Processing %zu candidates (approach: %.0f~%.0f cm)...",
                total, MIN_APPROACH_DISTANCE * 100, MAX_APPROACH_DISTANCE * 100);

    // ==========================================
    // 2-1. Planning Scene 스냅샷 (충돌 검사용)
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

    // 충돌 검사 콜백 (IK 해가 충돌 상태이면 거부)
    moveit::core::GroupStateValidityCallbackFn validity_callback;
    if (current_scene) {
      validity_callback = [&current_scene, this](
          moveit::core::RobotState* state,
          const moveit::core::JointModelGroup* group,
          const double* joint_values) -> bool {
        state->setJointGroupPositions(group, joint_values);
        state->update();
        bool in_collision = current_scene->isStateColliding(*state, group->getName());
        if (in_collision) {
          RCLCPP_DEBUG(get_logger(), "[BioIK] Candidate IK solution in collision, rejecting.");
        }
        return !in_collision;
      };
    }

    // 팔꿈치 링크 이름 및 Y축 부호 설정
    //   right arm: +Y축이 중력(아래) 방향 → y_sign = +1.0
    //   left  arm: -Y축이 중력(아래) 방향 → y_sign = -1.0
    std::string elbow_link = (request->arm_id == "left") ? "link_left_arm_3" : "link_right_arm_3";
    double y_sign = (request->arm_id == "left") ? -1.0 : 1.0;

    // ==========================================
    // 팔꿈치 cost 평가 함수 (IK solver cost_fn 과 배치 정렬 모두에 사용)
    // ==========================================
    const auto compute_elbow_cost = [&elbow_link, y_sign](
        const moveit::core::RobotState& state) -> double {
      const Eigen::Isometry3d& elbow_tf = state.getGlobalLinkTransform(elbow_link);
      Eigen::Vector3d local_x = elbow_tf.rotation().col(0);
      Eigen::Vector3d local_y = elbow_tf.rotation().col(1);
      Eigen::Vector3d local_z = elbow_tf.rotation().col(2);
      Eigen::Vector3d gravity(0.0, 0.0, -1.0);

      // [조건1] Y축 방향 (right: +Y→중력, left: -Y→중력)
      double cost_y = 1.0 - y_sign * local_y.dot(gravity);

      // [조건2] X,Z축이 수평면에 있어야 함
      double cost_xz = local_x.z() * local_x.z() + local_z.z() * local_z.z();

      return cost_y + cost_xz;
    };

    // IK solver에 전달할 cost function (weight 적용)
    const double elbow_cost_weight = 0.001;
    const auto elbow_cost_fn = [&compute_elbow_cost, elbow_cost_weight](
        const geometry_msgs::msg::Pose& /*goal_pose*/,
        const moveit::core::RobotState& solution_state,
        const moveit::core::JointModelGroup* /*jmg*/,
        const std::vector<double>& /*seed_state*/) -> double {
      return elbow_cost_weight * compute_elbow_cost(solution_state);
    };

    // ==========================================
    // 3. 배치 처리: BATCH_SIZE개씩 IK → cost 정렬 → planning
    // ==========================================
    const size_t BATCH_SIZE = 5;

    for (size_t batch_start = 0; batch_start < total; batch_start += BATCH_SIZE) {
      size_t batch_end = std::min(batch_start + BATCH_SIZE, total);

      RCLCPP_INFO(get_logger(), "==============================");
      RCLCPP_INFO(get_logger(), "[Batch] Processing candidates %zu~%zu / %zu",
                  batch_start + 1, batch_end, total);

      // ------------------------------------------
      // [Phase 1] 배치 내 모든 후보에 대해 IK 계산 + cost 평가
      // ------------------------------------------
      struct IkCandidate {
        size_t original_index;       // 원래 poses 배열의 인덱스
        moveit::core::RobotStatePtr state;  // IK 해
        geometry_msgs::msg::Pose pose_world; // 변환된 포즈
        double elbow_cost;           // 팔꿈치 cost (낮을수록 좋음)
      };
      std::vector<IkCandidate> ik_candidates;

      for (size_t i = batch_start; i < batch_end; ++i) {
        moveit::core::RobotStatePtr target_state = active_group->getCurrentState();
        const auto* jmg_ik = target_state->getJointModelGroup(active_group->getName());

        // 프레임 변환
        geometry_msgs::msg::Pose target_pose_world;
        {
          Eigen::Isometry3d pose_in_input;
          pose_in_input.setIdentity();
          pose_in_input.translation() = Eigen::Vector3d(
              poses[i].position.x, poses[i].position.y, poses[i].position.z);
          pose_in_input.linear() = Eigen::Quaterniond(
              poses[i].orientation.w, poses[i].orientation.x,
              poses[i].orientation.y, poses[i].orientation.z).toRotationMatrix();

          Eigen::Isometry3d pose_world;
          if (input_frame != planning_frame && target_state->knowsFrameTransform(input_frame)) {
            Eigen::Isometry3d frame_tf = target_state->getFrameTransform(input_frame);
            pose_world = frame_tf * pose_in_input;
          } else {
            pose_world = pose_in_input;
          }

          target_pose_world.position.x = pose_world.translation().x();
          target_pose_world.position.y = pose_world.translation().y();
          target_pose_world.position.z = pose_world.translation().z();
          Eigen::Quaterniond q_w(pose_world.rotation());
          target_pose_world.orientation.x = q_w.x();
          target_pose_world.orientation.y = q_w.y();
          target_pose_world.orientation.z = q_w.z();
          target_pose_world.orientation.w = q_w.w();
        }

        // IK 계산
        kinematics::KinematicsQueryOptions ik_opts;
        ik_opts.return_approximate_solution = true;

        bool found_ik = target_state->setFromIK(
            jmg_ik,
            target_pose_world,
            target_ee_link,
            0.1,
            validity_callback,
            ik_opts,
            elbow_cost_fn
        );

        if (!found_ik) {
          RCLCPP_INFO(get_logger(), "[IK] Candidate %zu: FAILED", i + 1);
          continue;
        }

        double cost = compute_elbow_cost(*target_state);
        RCLCPP_INFO(get_logger(), "[IK] Candidate %zu: OK (elbow cost = %.4f)", i + 1, cost);

        ik_candidates.push_back({i, target_state, target_pose_world, cost});
      }

      if (ik_candidates.empty()) {
        RCLCPP_WARN(get_logger(), "[Batch] No IK solutions in this batch. Next batch.");
        continue;
      }

      // ------------------------------------------
      // [Phase 2] cost 오름차순 정렬 (낮은 cost = 좋은 팔꿈치 자세)
      // ------------------------------------------
      std::sort(ik_candidates.begin(), ik_candidates.end(),
          [](const IkCandidate& a, const IkCandidate& b) {
            return a.elbow_cost < b.elbow_cost;
          });

      RCLCPP_INFO(get_logger(), "[Batch] %zu IK solutions found. Sorted by elbow cost:", ik_candidates.size());
      for (size_t r = 0; r < ik_candidates.size(); ++r) {
        RCLCPP_INFO(get_logger(), "  Rank %zu: candidate %zu (cost=%.4f)",
                    r + 1, ik_candidates[r].original_index + 1, ik_candidates[r].elbow_cost);
      }

      // ------------------------------------------
      // [Phase 3] 정렬된 순서대로 Step1 + Step2 planning 시도
      // ------------------------------------------
      for (auto& cand : ik_candidates) {
        RCLCPP_INFO(get_logger(), "------------------------------");
        RCLCPP_INFO(get_logger(), "[Plan] Trying candidate %zu (elbow cost=%.4f)",
                    cand.original_index + 1, cand.elbow_cost);

        // Step 1: Joint-space planning
        active_group->setStartStateToCurrentState();
        active_group->setJointValueTarget(*cand.state);

        moveit::planning_interface::MoveGroupInterface::Plan step1_plan;
        auto step1_result = active_group->plan(step1_plan);

        if (step1_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "[Step1 Plan] FAILED: %s. Trying next ranked.",
                      errorCodeToString(step1_result.val).c_str());
          continue;
        }
        RCLCPP_INFO(get_logger(), "[Step1 Plan] SUCCESS (%zu points)",
                    step1_plan.trajectory_.joint_trajectory.points.size());

        // Step 2: Cartesian approach
        auto step1_end_state = buildStateFromTrajectoryEnd(active_group, step1_plan.trajectory_);
        const auto* jmg = step1_end_state->getJointModelGroup(active_group->getName());

        bool acm_modified = setOctomapCollisionAllowed(gripper_links, true);

        auto waypoints = computeEefApproachWaypoints(step1_end_state, target_ee_link, LONG_APPROACH_DISTANCE);
        active_group->setStartState(*step1_end_state);

        moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
        moveit_msgs::msg::MoveItErrorCodes error_code;

        double fraction = active_group->computeCartesianPath(
          waypoints, 0.002, 0.0, cartesian_trajectory, true, &error_code);

        RCLCPP_INFO(get_logger(), "[Step2 Plan] Cartesian: %.1f%% coverage, %zu points",
                    fraction * 100.0, cartesian_trajectory.joint_trajectory.points.size());

        if (cartesian_trajectory.joint_trajectory.points.size() < 2) {
          RCLCPP_WARN(get_logger(), "[Step2 Plan] Not enough points. Trying next ranked.");
          if (acm_modified) setOctomapCollisionAllowed(gripper_links, false);
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
          if (acm_modified) setOctomapCollisionAllowed(gripper_links, false);
          continue;
        }

        scaleTrajectorySpeed(trimmed_step2, CARTESIAN_SPEED_SCALE);

        RCLCPP_INFO(get_logger(), "==> Step1 + Step2 planning SUCCEEDED for candidate %zu (elbow cost=%.4f)",
                    cand.original_index + 1, cand.elbow_cost);

        // ------------------------------------------
        // [RViz 전용 모드] 계획만 수행, 실제 로봇 실행 안함
        // 실제 실행을 원하면 아래 #if 0 을 #if 1 로 변경
        // ------------------------------------------
#if 0
        // Step 1 실행
        RCLCPP_INFO(get_logger(), "[Step1 Exec] Moving to pre-grasp...");
        active_group->setStartStateToCurrentState();
        auto exec1_result = active_group->execute(step1_plan);

        if (exec1_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "[Step1 Exec] FAILED: %s. Trying next ranked.",
                      errorCodeToString(exec1_result.val).c_str());
          if (acm_modified) setOctomapCollisionAllowed(gripper_links, false);
          continue;
        }
        RCLCPP_INFO(get_logger(), "[Step1 Exec] Done. Waiting for state sync...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // Step 2 실행
        RCLCPP_INFO(get_logger(), "[Step2 Exec] Cartesian approach (%.4f m)...", actual_distance);
        active_group->setStartStateToCurrentState();
        auto exec2_result = active_group->execute(trimmed_step2);

        if (exec2_result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "[Step2 Exec] FAILED: %s. Trying next ranked.",
                      errorCodeToString(exec2_result.val).c_str());
          if (acm_modified) setOctomapCollisionAllowed(gripper_links, false);
          continue;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));
        geometry_msgs::msg::PoseStamped final_pose = active_group->getCurrentPose(target_ee_link);
        RCLCPP_INFO(get_logger(), "[Done] Final EEF: [%.4f, %.4f, %.4f]",
                    final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z);
#endif

        RCLCPP_INFO(get_logger(), "[RViz Only] Planning succeeded. Skipping execution. "
                    "Check RViz 'Planned Path' display to visualize.");

        // ACM 복원
        if (acm_modified) {
          setOctomapCollisionAllowed(gripper_links, false);
        }

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
                          + " planned (elbow cost=" + std::to_string(cand.elbow_cost)
                          + ", RViz only).";
        RCLCPP_INFO(get_logger(), "==> Plan complete for candidate %zu. No execution.",
                    cand.original_index + 1);
        return;

      } // end ranked candidate loop

      RCLCPP_WARN(get_logger(), "[Batch] All ranked candidates failed planning. Next batch.");

    } // end batch loop

    response->success = false;
    response->message = "All candidates failed for arm: " + request->arm_id;
    RCLCPP_WARN(get_logger(), "All candidates exhausted.");
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

