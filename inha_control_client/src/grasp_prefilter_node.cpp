// =====================================================================
//  grasp_prefilter_node.cpp
//
//  Grasp 후보 사전 필터링 노드 (rby1_action_server 의 yaw flip 및
//  rigid-block pre-filter 책임을 분리한 노드).
//
//  파이프라인:
//    [graspgen_inference]  (geometry_msgs/PoseArray 발행)
//        │  service /execute_grasp_service (ExecuteGraspSrv)
//        ▼
//    [grasp_prefilter_node]   ← 이 노드
//        │  service /execute_grasp_planning (ExecuteGraspSrv)
//        ▼
//    [rby1_action_server]
//
//  본 노드의 단일 ExecuteGraspSrv 서비스 콜백에서 다음을 수행한다:
//    1. 입력 PoseArray 후보 n 개를 planning frame(world) 으로 변환
//    2. 각 후보를 [원본, EE 로컬 Z 180° flip] 두 자세로 복제 → 2n 개
//       인터리빙 순서로 [c1, c1*, c2, c2*, ...] 보존
//    3. PlanningScene 한 번 조회해서 octomap 추출
//    4. 2n 개 각각에 대해 rigid-block pre-filter 수행:
//         - 그리퍼 끝부분 24 점 (손가락 6 + EE 본체 2 + arm_5 16)
//         - 손목 카메라 콜리전 박스 8 코너점
//       한 점이라도 octomap 에 occupied 면 거부
//    5. 통과한 자세들을 카메라 자세 점수로 정렬 (위 향함 우선)
//       동점/동일 점수면 입력 순서 보존
//    6. 정렬된 PoseArray 를 action_server 의 ExecuteGraspSrv 로 전달
//       응답 success/message 를 그대로 호출자에게 반환
// =====================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rigid-block pre-filter: octomap 점 쿼리
#include <geometric_shapes/shapes.h>
#include <octomap/OcTree.h>

// TF2: 입력 frame → planning frame 변환
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>

#include "inha_interfaces/srv/execute_grasp_srv.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>
#include <cmath>

using ExecuteGraspSrv = inha_interfaces::srv::ExecuteGraspSrv;

class GraspPrefilterNode : public rclcpp::Node
{
public:
  explicit GraspPrefilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("grasp_prefilter_node", options)
  {
    // ------------------------------------------------------------------
    // 파라미터
    // ------------------------------------------------------------------
    // 입력 서비스 이름 (manager 가 호출하는 측, 기존 action_server 와 동일 이름 유지)
    this->declare_parameter<std::string>("input_service_name", "/execute_grasp_planning");
    // 다음 노드 호출 서비스 이름.
    //   기본값을 /execute_grasp_curobo 로 둬서 curobo_planner_node 와 직결.
    //   기존 rby1_action_server (MoveIt) 로 보내고 싶으면 launch 에서
    //   downstream_service_name:=/execute_grasp_planning 으로 override.
    this->declare_parameter<std::string>("downstream_service_name", "/execute_grasp_curobo");
    // PlanningScene 서비스 (MoveIt 표준)
    this->declare_parameter<std::string>("planning_scene_service", "/get_planning_scene");
    // robot_description 파라미터에서 모델 로드 (planning frame 결정 용)
    this->declare_parameter<std::string>("robot_description", "");
    // 다음 노드 호출 timeout [sec]
    this->declare_parameter<double>("downstream_timeout_sec", 120.0);
    // PlanningScene 호출 timeout [sec]
    this->declare_parameter<double>("scene_timeout_sec", 5.0);
    // rigid-block pre-filter 활성화 (false 면 카메라 자세 정렬만 적용)
    this->declare_parameter<bool>("enable_rigid_block", true);
    // 카메라 콜리전 검사 활성화 (false 면 그리퍼 끝부분만 검사)
    this->declare_parameter<bool>("enable_camera_check", true);

    input_service_name_      = this->get_parameter("input_service_name").as_string();
    downstream_service_name_ = this->get_parameter("downstream_service_name").as_string();
    planning_scene_service_  = this->get_parameter("planning_scene_service").as_string();
    downstream_timeout_sec_  = this->get_parameter("downstream_timeout_sec").as_double();
    scene_timeout_sec_       = this->get_parameter("scene_timeout_sec").as_double();
    enable_rigid_block_      = this->get_parameter("enable_rigid_block").as_bool();
    enable_camera_check_     = this->get_parameter("enable_camera_check").as_bool();

    // ------------------------------------------------------------------
    // RobotModel 로드
    //   planning frame 과 EE/카메라 링크 transform 을 알기 위해 필요.
    //   robot_description 파라미터(global) 가 보통 robot_state_publisher 에
    //   의해 발행되어 있으므로 이를 받아 모델을 구성한다.
    // ------------------------------------------------------------------
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    {
      robot_model_loader::RobotModelLoader::Options opts;
      opts.robot_description_ = "robot_description";
      opts.load_kinematics_solvers_ = false;
      auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(
          shared_from_this_safe(), opts);
      robot_model_ = loader->getModel();
      if (!robot_model_) {
        RCLCPP_ERROR(get_logger(), "Failed to load robot model from /robot_description.");
      } else {
        planning_frame_ = robot_model_->getModelFrame();
        RCLCPP_INFO(get_logger(),
                    "Robot model loaded. planning_frame='%s'", planning_frame_.c_str());
      }
    }

    // 카메라 링크 EE 로컬 transform 캐시 (URDF 의 fixed joint 값으로부터 계산)
    cacheCameraTransforms();

    // ------------------------------------------------------------------
    // TF
    // ------------------------------------------------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ------------------------------------------------------------------
    // PlanningScene 클라이언트
    // ------------------------------------------------------------------
    get_planning_scene_client_ = this->create_client<moveit_msgs::srv::GetPlanningScene>(
        planning_scene_service_, rmw_qos_profile_services_default, callback_group_);

    // ------------------------------------------------------------------
    // 다음 노드(action_server) 호출 클라이언트
    // ------------------------------------------------------------------
    downstream_client_ = this->create_client<ExecuteGraspSrv>(
        downstream_service_name_, rmw_qos_profile_services_default, callback_group_);

    // ------------------------------------------------------------------
    // 본 노드의 ExecuteGraspSrv 서비스 서버 (manager 가 호출하는 측)
    // ------------------------------------------------------------------
    service_server_ = this->create_service<ExecuteGraspSrv>(
        input_service_name_,
        std::bind(&GraspPrefilterNode::handleService, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    // 디버그: 거부/통과 자세 시각화용 PoseArray 발행
    rejected_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/grasp_prefilter/rejected", 10);
    accepted_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/grasp_prefilter/accepted", 10);
    // 거부된 후보의 32개 샘플 포인트 시각화 (RViz MarkerArray)
    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grasp_prefilter/debug_markers", 10);

    RCLCPP_INFO(get_logger(),
        "✅ grasp_prefilter_node ready. "
        "in='%s'  out='%s'  rigid_block=%s  camera=%s",
        input_service_name_.c_str(), downstream_service_name_.c_str(),
        enable_rigid_block_ ? "ON" : "OFF",
        enable_camera_check_ ? "ON" : "OFF");
  }

private:
  // shared_from_this 를 생성자 중에 안전하게 쓰기 위한 우회.
  // RobotModelLoader 에 weak_ptr 를 전달해야 하는데 생성자 내부에서는
  // shared_from_this 가 유효하지 않으므로, RobotModelLoader 가 노드 소유권을
  // 잡지 않는 형태로만 호출하기 위해 임시 lambda owner 를 사용한다.
  std::shared_ptr<rclcpp::Node> shared_from_this_safe()
  {
    // 생성자 시점에는 enable_shared_from_this 의 weak_ptr 가 비어있을 수 있다.
    // 안전하게 비-소유 shared_ptr 로 노드를 감싼다 (deleter no-op).
    return std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
  }

  // ==================================================================
  // 카메라 링크 EE 로컬 transform 캐시
  //   URDF 의 fixed joint 값을 RobotModel 로부터 읽어와 EE 로컬에서
  //   카메라 콜리전 박스의 8 코너 좌표를 미리 계산해 둔다.
  // ==================================================================
  void cacheCameraTransforms()
  {
    if (!robot_model_) return;

    auto cache_one = [&](const std::string& ee_link,
                          const std::string& cam_link,
                          std::vector<Eigen::Vector3d>& corners_out)
    {
      corners_out.clear();
      const auto* ee_lm  = robot_model_->getLinkModel(ee_link);
      const auto* cam_lm = robot_model_->getLinkModel(cam_link);
      if (!ee_lm || !cam_lm) {
        RCLCPP_WARN(get_logger(),
            "[CamCache] Missing link: ee='%s' (%s), cam='%s' (%s).",
            ee_link.c_str(),  ee_lm  ? "OK" : "MISSING",
            cam_link.c_str(), cam_lm ? "OK" : "MISSING");
        return;
      }

      // RobotModel 은 link 들의 transform 을 default joint values 기준으로 가지고 있다.
      // 카메라는 ee 에 대해 fixed 이므로 default transform 이 곧 EE-local transform.
      moveit::core::RobotState rs(robot_model_);
      rs.setToDefaultValues();
      rs.update();

      Eigen::Isometry3d T_ee  = rs.getGlobalLinkTransform(ee_lm);
      Eigen::Isometry3d T_cam = rs.getGlobalLinkTransform(cam_lm);
      Eigen::Isometry3d T_ee_to_cam = T_ee.inverse() * T_cam;

      // URDF 박스 크기는 캐시 시점에 직접 알기 어려우므로, 알려진 값으로 사용.
      // (rby1_inha.urdf 의 camera_*_link collision: box size="0.032 0.042 0.087")
      const double bx = 0.032, by = 0.042, bz = 0.087;
      static const double kSafetyShrink = 0.95;  // 안쪽으로 약간 마진
      const double hx = bx * 0.5 * kSafetyShrink;
      const double hy = by * 0.5 * kSafetyShrink;
      const double hz = bz * 0.5 * kSafetyShrink;

      const Eigen::Vector3d local_corners[8] = {
        { hx,  hy,  hz}, { hx,  hy, -hz}, { hx, -hy,  hz}, { hx, -hy, -hz},
        {-hx,  hy,  hz}, {-hx,  hy, -hz}, {-hx, -hy,  hz}, {-hx, -hy, -hz}
      };
      corners_out.reserve(8);
      for (const auto& c : local_corners) {
        Eigen::Vector3d p_ee = T_ee_to_cam * c;
        corners_out.push_back(p_ee);
      }

      RCLCPP_INFO(get_logger(),
          "[CamCache] %s → %s: T_ee_to_cam.t=[%.4f, %.4f, %.4f], 8 corners cached.",
          ee_link.c_str(), cam_link.c_str(),
          T_ee_to_cam.translation().x(),
          T_ee_to_cam.translation().y(),
          T_ee_to_cam.translation().z());
    };

    cache_one("ee_right", "camera_right_link", camera_corners_local_right_);
    cache_one("ee_left",  "camera_left_link",  camera_corners_local_left_);

    // 카메라 방향 벡터 (EE 로컬에서 본 카메라가 가리키는 대략적 방향).
    // URDF joint origin 의 부호로부터: right 는 -Y, left 는 +Y 쪽에 카메라가 있다.
    // 이 벡터의 world Z 성분이 양수이면 "카메라가 위를 향함" 으로 정의.
    camera_local_dir_right_ = Eigen::Vector3d(0.0, -1.0, 0.0);
    camera_local_dir_left_  = Eigen::Vector3d(0.0,  1.0, 0.0);
  }

  // ==================================================================
  // 샘플 포인트 카테고리 (시각화 색상 분류용)
  // ==================================================================
  enum class SampleCat {
    Finger,   // 손가락 (빨강)
    EeBody,   // EE 본체 (주황)
    Arm5,     // arm_5 cylinder (노랑)
    Camera    // 손목 카메라 (파랑)
  };

  // ==================================================================
  // 그리퍼 끝부분 24 점 (EE local frame)
  //   기존 rby1_action_server_bio_ik.cpp 의 checkRigidBlockCollision 의
  //   sample 정의를 그대로 이전. label 외에 카테고리도 포함.
  // ==================================================================
  struct SamplePt {
    double x, y, z;
    const char* label;
    SampleCat cat;
  };
  static const std::vector<SamplePt>& gripperSamples()
  {
    static const std::vector<SamplePt> kSamples = {
      // Finger tips
      { 0.05,  0.0, -0.135, "finger1_tip",   SampleCat::Finger},
      {-0.05,  0.0, -0.135, "finger2_tip",   SampleCat::Finger},
      // Finger centers
      { 0.05,  0.0, -0.105, "finger1_mid",   SampleCat::Finger},
      {-0.05,  0.0, -0.105, "finger2_mid",   SampleCat::Finger},
      // Finger bases
      { 0.05,  0.0, -0.075, "finger1_base",  SampleCat::Finger},
      {-0.05,  0.0, -0.075, "finger2_base",  SampleCat::Finger},
      // EE center
      { 0.0,   0.0, -0.03,  "ee_low",        SampleCat::EeBody},
      { 0.0,   0.0,  0.0,   "ee_center",     SampleCat::EeBody},
      // arm_5 cylinder: axis samples
      { 0.0,   0.0, -0.06,  "arm5_bot",      SampleCat::Arm5},
      { 0.0,   0.0,  0.0,   "arm5_z0",       SampleCat::Arm5},
      { 0.0,   0.0,  0.03,  "arm5_z1",       SampleCat::Arm5},
      { 0.0,   0.0,  0.06,  "arm5_z2",       SampleCat::Arm5},
      { 0.0,   0.0,  0.09,  "arm5_z3",       SampleCat::Arm5},
      { 0.0,   0.0,  0.116, "arm5_top",      SampleCat::Arm5},
      // arm_5 cylinder: radial samples (R=0.045)
      { 0.045,  0.0,   0.0,   "arm5_r+x_0",  SampleCat::Arm5},
      {-0.045,  0.0,   0.0,   "arm5_r-x_0",  SampleCat::Arm5},
      { 0.0,    0.045, 0.0,   "arm5_r+y_0",  SampleCat::Arm5},
      { 0.0,   -0.045, 0.0,   "arm5_r-y_0",  SampleCat::Arm5},
      { 0.045,  0.0,   0.06,  "arm5_r+x_1",  SampleCat::Arm5},
      {-0.045,  0.0,   0.06,  "arm5_r-x_1",  SampleCat::Arm5},
      { 0.0,    0.045, 0.06,  "arm5_r+y_1",  SampleCat::Arm5},
      { 0.0,   -0.045, 0.06,  "arm5_r-y_1",  SampleCat::Arm5},
      { 0.045,  0.0,   0.116, "arm5_r+x_t",  SampleCat::Arm5},
      {-0.045,  0.0,   0.116, "arm5_r-x_t",  SampleCat::Arm5},
    };
    return kSamples;
  }

  // ==================================================================
  // 디버그용 샘플 결과 구조체
  //   각 후보의 모든 샘플 점에 대해 카테고리/라벨/world좌표/occupied 여부를
  //   호출자에게 돌려줘 RViz 시각화에 사용한다.
  // ==================================================================
  struct SampleResult {
    Eigen::Vector3d pt_world;
    SampleCat cat;
    std::string label;
    bool occupied;
  };

  // ==================================================================
  // Rigid-block + 카메라 콜리전 검사
  //   ee_pose_world (planning frame) 이 주어지면:
  //     - 그리퍼 24 점을 world 로 옮겨 octree 점유 검사
  //     - (옵션) 카메라 8 코너점도 함께 검사
  //   반환:
  //     true  = 충돌 발견 → 거부
  //     false = 통과
  //   detail_out: 첫 hit 라벨 (없으면 빈 문자열)
  //   results_out: (옵션, nullptr 가능) 모든 샘플의 결과를 채워서 반환.
  //                 nullptr 일 때는 첫 hit 시 즉시 반환(빠름).
  //                 비-nullptr 일 때는 전 샘플을 모두 검사(시각화용).
  // ==================================================================
  bool checkRigidBlockCollision(
      const geometry_msgs::msg::Pose& ee_pose_world,
      const planning_scene::PlanningScenePtr& scene,
      const std::string& arm_id,
      std::string& detail_out,
      std::vector<SampleResult>* results_out) const
  {
    detail_out.clear();
    if (results_out) results_out->clear();

    if (!scene) return false;  // scene 없으면 필터 비활성

    const auto& world = scene->getWorld();
    auto octomap_obj = world->getObject("<octomap>");
    if (!octomap_obj || octomap_obj->shapes_.empty()) {
      // octomap 없으면 통과 (검사 불가)
      return false;
    }
    auto octree_shape = std::dynamic_pointer_cast<const shapes::OcTree>(octomap_obj->shapes_[0]);
    if (!octree_shape || !octree_shape->octree) {
      return false;
    }
    const auto& octree = octree_shape->octree;

    Eigen::Isometry3d octomap_tf = Eigen::Isometry3d::Identity();
    if (!octomap_obj->shape_poses_.empty()) {
      octomap_tf = octomap_obj->shape_poses_[0];
    }
    Eigen::Isometry3d octomap_tf_inv = octomap_tf.inverse();

    Eigen::Isometry3d T_ee = Eigen::Isometry3d::Identity();
    T_ee.translation() = Eigen::Vector3d(
        ee_pose_world.position.x, ee_pose_world.position.y, ee_pose_world.position.z);
    T_ee.linear() = Eigen::Quaterniond(
        ee_pose_world.orientation.w, ee_pose_world.orientation.x,
        ee_pose_world.orientation.y, ee_pose_world.orientation.z).toRotationMatrix();

    bool any_occupied = false;

    auto query_one = [&](const Eigen::Vector3d& pt_local,
                          const std::string& label,
                          SampleCat cat) -> bool {
      Eigen::Vector3d pt_world = T_ee * pt_local;
      Eigen::Vector3d pt_oct   = octomap_tf_inv * pt_world;
      auto node = octree->search(pt_oct.x(), pt_oct.y(), pt_oct.z());
      bool occ = (node && octree->isNodeOccupied(node));
      if (results_out) {
        SampleResult r;
        r.pt_world = pt_world;
        r.cat      = cat;
        r.label    = label;
        r.occupied = occ;
        results_out->push_back(std::move(r));
      }
      if (occ) {
        if (detail_out.empty()) detail_out = label;
        any_occupied = true;
      }
      return occ;
    };

    // 그리퍼 24 점
    for (const auto& s : gripperSamples()) {
      bool occ = query_one(Eigen::Vector3d(s.x, s.y, s.z), s.label, s.cat);
      // results_out 가 nullptr 이면 첫 hit 즉시 반환(빠름)
      if (occ && !results_out) return true;
    }

    // 카메라 8 코너점
    if (enable_camera_check_) {
      const auto& corners =
          (arm_id == "right") ? camera_corners_local_right_
                              : camera_corners_local_left_;
      for (size_t i = 0; i < corners.size(); ++i) {
        std::string label = "cam_corner_" + std::to_string(i);
        bool occ = query_one(corners[i], label, SampleCat::Camera);
        if (occ && !results_out) return true;
      }
    }

    return any_occupied;
  }

  // ==================================================================
  // PlanningScene 한 번 가져오기
  // ==================================================================
  planning_scene::PlanningScenePtr fetchPlanningScene()
  {
    if (!robot_model_) {
      RCLCPP_ERROR_ONCE(get_logger(), "[Scene] No robot model.");
      return nullptr;
    }
    if (!get_planning_scene_client_->wait_for_service(
          std::chrono::duration<double>(scene_timeout_sec_))) {
      RCLCPP_WARN(get_logger(), "[Scene] PlanningScene service not available.");
      return nullptr;
    }

    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE;

    auto future = get_planning_scene_client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::duration<double>(scene_timeout_sec_));
    if (status != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "[Scene] PlanningScene call timed out.");
      return nullptr;
    }
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    scene->usePlanningSceneMsg(future.get()->scene);
    return scene;
  }

  // ==================================================================
  // PoseArray 의 모든 pose 를 입력 frame → planning frame 으로 변환
  // ==================================================================
  bool transformPosesToPlanningFrame(
      const geometry_msgs::msg::PoseArray& in,
      std::vector<geometry_msgs::msg::Pose>& out_world)
  {
    out_world.clear();
    out_world.reserve(in.poses.size());

    std::string in_frame = in.header.frame_id;
    if (in_frame.empty()) {
      RCLCPP_WARN(get_logger(),
          "[Frame] Input PoseArray has empty frame_id; assuming planning_frame='%s'.",
          planning_frame_.c_str());
      for (const auto& p : in.poses) out_world.push_back(p);
      return true;
    }

    if (in_frame == planning_frame_) {
      for (const auto& p : in.poses) out_world.push_back(p);
      return true;
    }

    // TF 조회 (한 번)
    geometry_msgs::msg::TransformStamped tf_in_to_planning;
    try {
      tf_in_to_planning = tf_buffer_->lookupTransform(
          planning_frame_, in_frame, tf2::TimePointZero,
          tf2::durationFromSec(2.0));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
          "[Frame] TF lookup '%s' → '%s' failed: %s",
          in_frame.c_str(), planning_frame_.c_str(), e.what());
      return false;
    }

    for (const auto& p : in.poses) {
      geometry_msgs::msg::Pose p_world;
      tf2::doTransform(p, p_world, tf_in_to_planning);
      out_world.push_back(p_world);
    }
    RCLCPP_INFO(get_logger(),
        "[Frame] Transformed %zu poses '%s' → '%s'.",
        out_world.size(), in_frame.c_str(), planning_frame_.c_str());
    return true;
  }

  // ==================================================================
  // 카메라 자세 점수
  //   = (R_eef * camera_local_dir).z
  //   양수가 클수록 카메라가 world 위(+Z)를 향함. 우선순위 높음.
  // ==================================================================
  double cameraScore(const geometry_msgs::msg::Pose& p, const std::string& arm_id) const
  {
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    const Eigen::Vector3d& cam_local =
        (arm_id == "right") ? camera_local_dir_right_ : camera_local_dir_left_;
    Eigen::Vector3d cam_world = R * cam_local;
    return cam_world.z();
  }

  // ==================================================================
  // EE 로컬 Z (=approach) 축 기준 180° flip (right-multiply)
  // ==================================================================
  geometry_msgs::msg::Pose flipYaw180(const geometry_msgs::msg::Pose& p) const
  {
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Matrix3d R_yaw180 =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d R_new = q.toRotationMatrix() * R_yaw180;
    Eigen::Quaterniond q_new(R_new);
    q_new.normalize();

    geometry_msgs::msg::Pose out = p;  // 위치 보존
    out.orientation.w = q_new.w();
    out.orientation.x = q_new.x();
    out.orientation.y = q_new.y();
    out.orientation.z = q_new.z();
    return out;
  }

  // ==================================================================
  // 디버그 시각화: MarkerArray 빌더
  //
  //   카테고리(Finger/EeBody/Arm5/Camera) × 상태(occupied/clear) =
  //   8개의 SPHERE_LIST 마커로 거부된 후보들의 모든 샘플 점을 한 번에 표시.
  //   각 거부 후보 위에 "#42* (cam_corner_3)" 같은 식별 텍스트 추가.
  //
  //   매 service call 첫 머리에 DELETEALL 액션을 한 번 보내 이전 frame 의
  //   잔재 마커를 깨끗이 지운다. 거부 후보가 0 개여도 DELETEALL 만 발행해
  //   화면을 클리어한다.
  // ==================================================================
  struct RejectedCand {
    size_t orig_index;
    bool flipped;
    std::string first_hit_label;
    geometry_msgs::msg::Pose pose;
    std::vector<SampleResult> samples;
  };

  std_msgs::msg::ColorRGBA makeColor(double r, double g, double b, double a) const
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
  }

  std_msgs::msg::ColorRGBA categoryColor(SampleCat cat, bool occupied) const
  {
    // occupied 면 진한 채도 + alpha=1.0, clear 면 옅은 채도 + alpha=0.3
    const double a = occupied ? 1.0 : 0.3;
    switch (cat) {
      case SampleCat::Finger:  return makeColor(1.0, 0.15, 0.15, a);  // 빨강
      case SampleCat::EeBody:  return makeColor(1.0, 0.55, 0.10, a);  // 주황
      case SampleCat::Arm5:    return makeColor(1.0, 0.95, 0.10, a);  // 노랑
      case SampleCat::Camera:  return makeColor(0.20, 0.55, 1.0,  a); // 파랑
    }
    return makeColor(0.5, 0.5, 0.5, a);
  }

  const char* categoryName(SampleCat cat) const
  {
    switch (cat) {
      case SampleCat::Finger: return "finger";
      case SampleCat::EeBody: return "ee_body";
      case SampleCat::Arm5:   return "arm5";
      case SampleCat::Camera: return "camera";
    }
    return "unknown";
  }

  void publishDebugMarkers(const std::vector<RejectedCand>& rejected,
                            const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray arr;

    // (1) DELETEALL 한 번
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = stamp;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(m);
    }

    // (2) 카테고리×상태 별 SPHERE_LIST 8 개 (rejected 가 비어있어도 OK)
    struct Bucket {
      SampleCat cat; bool occupied; const char* ns;
      double scale;
    };
    static const Bucket kBuckets[] = {
      {SampleCat::Finger, true,  "finger_occ",  0.012},
      {SampleCat::Finger, false, "finger_clr",  0.006},
      {SampleCat::EeBody, true,  "eebody_occ",  0.012},
      {SampleCat::EeBody, false, "eebody_clr",  0.006},
      {SampleCat::Arm5,   true,  "arm5_occ",    0.012},
      {SampleCat::Arm5,   false, "arm5_clr",    0.006},
      {SampleCat::Camera, true,  "camera_occ",  0.014},
      {SampleCat::Camera, false, "camera_clr",  0.007},
    };

    int marker_id = 0;
    for (const auto& bk : kBuckets) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = stamp;
      m.ns = bk.ns;
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = bk.scale;
      m.scale.y = bk.scale;
      m.scale.z = bk.scale;
      m.color = categoryColor(bk.cat, bk.occupied);
      m.lifetime = rclcpp::Duration::from_seconds(0);  // 다음 DELETEALL 까지 유지

      for (const auto& rc : rejected) {
        for (const auto& s : rc.samples) {
          if (s.cat != bk.cat || s.occupied != bk.occupied) continue;
          geometry_msgs::msg::Point p;
          p.x = s.pt_world.x();
          p.y = s.pt_world.y();
          p.z = s.pt_world.z();
          m.points.push_back(p);
          m.colors.push_back(m.color);  // per-point color (=동일색이지만 명시)
        }
      }
      arr.markers.push_back(m);
    }

    // (3) 거부 후보별 텍스트 라벨 (EE 위치 위에 "#42* (cam_corner_3)")
    for (const auto& rc : rejected) {
      visualization_msgs::msg::Marker t;
      t.header.frame_id = planning_frame_;
      t.header.stamp = stamp;
      t.ns = "label";
      t.id = marker_id++;
      t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::msg::Marker::ADD;
      t.pose = rc.pose;
      t.pose.position.z += 0.05;  // EE 위 5 cm 띄움
      t.scale.z = 0.025;          // 글자 높이
      t.color = makeColor(1.0, 1.0, 1.0, 0.9);
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "#%zu%s (%s)",
                    rc.orig_index + 1,
                    rc.flipped ? "*" : "",
                    rc.first_hit_label.c_str());
      t.text = buf;
      t.lifetime = rclcpp::Duration::from_seconds(0);
      arr.markers.push_back(t);
    }

    debug_markers_pub_->publish(arr);
  }

  // ==================================================================
  // 메인 콜백
  // ==================================================================
  void handleService(
      const std::shared_ptr<ExecuteGraspSrv::Request> request,
      std::shared_ptr<ExecuteGraspSrv::Response> response)
  {
    auto t_start = std::chrono::steady_clock::now();

    const std::string& arm_id = request->arm_id;
    if (arm_id != "left" && arm_id != "right") {
      response->success = false;
      response->message = "Invalid arm_id. Use 'left' or 'right'.";
      RCLCPP_WARN(get_logger(), "[Prefilter] %s", response->message.c_str());
      return;
    }
    const size_t n_in = request->grasp_candidates.poses.size();
    RCLCPP_INFO(get_logger(),
        "📥 [Prefilter] Request: arm=%s, candidates=%zu",
        arm_id.c_str(), n_in);

    if (n_in == 0) {
      response->success = false;
      response->message = "Empty grasp_candidates.";
      return;
    }

    // ------------------------------------------------------------------
    // 1. 입력 frame → planning frame 변환
    // ------------------------------------------------------------------
    std::vector<geometry_msgs::msg::Pose> world_poses_orig;
    if (!transformPosesToPlanningFrame(request->grasp_candidates, world_poses_orig)) {
      response->success = false;
      response->message = "TF transform failed.";
      return;
    }

    // ------------------------------------------------------------------
    // 2. 원본 + flip 인터리빙 → 2n
    //    duplicate 정책은 "둘 다 살림". 정렬은 카메라 점수로 후속 처리.
    // ------------------------------------------------------------------
    struct Variant {
      size_t orig_index;     // 원본 grasp 후보 인덱스
      bool   flipped;        // false = 원본, true = yaw flip
      geometry_msgs::msg::Pose pose;
      double cam_score;      // 카메라 자세 점수 (정렬용)
    };
    std::vector<Variant> variants;
    variants.reserve(2 * n_in);
    for (size_t i = 0; i < n_in; ++i) {
      Variant v_orig;
      v_orig.orig_index = i;
      v_orig.flipped    = false;
      v_orig.pose       = world_poses_orig[i];
      v_orig.cam_score  = cameraScore(v_orig.pose, arm_id);
      variants.push_back(v_orig);

      Variant v_flip;
      v_flip.orig_index = i;
      v_flip.flipped    = true;
      v_flip.pose       = flipYaw180(world_poses_orig[i]);
      v_flip.cam_score  = cameraScore(v_flip.pose, arm_id);
      variants.push_back(v_flip);
    }
    RCLCPP_INFO(get_logger(),
        "[Prefilter] Variants generated: %zu (original) × 2 = %zu",
        n_in, variants.size());

    // ------------------------------------------------------------------
    // 3. PlanningScene 한 번 fetch
    // ------------------------------------------------------------------
    planning_scene::PlanningScenePtr scene = nullptr;
    if (enable_rigid_block_) {
      scene = fetchPlanningScene();
      if (!scene) {
        RCLCPP_WARN(get_logger(),
            "[Prefilter] PlanningScene unavailable; rigid-block check will be skipped.");
      }
    }

    // ------------------------------------------------------------------
    // 4. 각 variant 에 대해 rigid-block + 카메라 검사
    //    거부된 후보의 모든 샘플 점을 함께 수집해 RViz 시각화에 사용.
    // ------------------------------------------------------------------
    std::vector<Variant> accepted;
    accepted.reserve(variants.size());

    geometry_msgs::msg::PoseArray rejected_msg, accepted_msg;
    const auto stamp = this->get_clock()->now();
    rejected_msg.header.stamp = stamp;
    rejected_msg.header.frame_id = planning_frame_;
    accepted_msg.header = rejected_msg.header;

    std::vector<RejectedCand> rejected_debug;
    rejected_debug.reserve(variants.size());

    // 카테고리별 occupied hit 카운트 (요약 로그용)
    size_t n_hit_finger = 0, n_hit_eebody = 0, n_hit_arm5 = 0, n_hit_camera = 0;

    size_t n_rejected_rigid = 0;
    for (const auto& v : variants) {
      bool reject = false;
      std::string detail;
      std::vector<SampleResult> samples;
      if (enable_rigid_block_ && scene) {
        // results_out 를 넘겨 전체 샘플을 검사 (시각화 용도)
        if (checkRigidBlockCollision(v.pose, scene, arm_id, detail, &samples)) {
          reject = true;
          ++n_rejected_rigid;
        }
      }
      if (reject) {
        rejected_msg.poses.push_back(v.pose);

        // 디버그용 누적 + 카테고리 카운트
        RejectedCand rc;
        rc.orig_index = v.orig_index;
        rc.flipped    = v.flipped;
        rc.first_hit_label = detail;
        rc.pose       = v.pose;
        rc.samples    = std::move(samples);
        for (const auto& s : rc.samples) {
          if (!s.occupied) continue;
          switch (s.cat) {
            case SampleCat::Finger: ++n_hit_finger; break;
            case SampleCat::EeBody: ++n_hit_eebody; break;
            case SampleCat::Arm5:   ++n_hit_arm5;   break;
            case SampleCat::Camera: ++n_hit_camera; break;
          }
        }
        rejected_debug.push_back(std::move(rc));

        RCLCPP_DEBUG(get_logger(),
            "  [Reject] cand=%zu%s  first_hit=%s",
            v.orig_index + 1, v.flipped ? "*" : "", detail.c_str());
      } else {
        accepted.push_back(v);
        accepted_msg.poses.push_back(v.pose);
      }
    }

    RCLCPP_INFO(get_logger(),
        "[Prefilter] Rigid-block: %zu/%zu rejected, %zu passed. "
        "(occupied hits: finger=%zu, ee_body=%zu, arm5=%zu, camera=%zu)",
        n_rejected_rigid, variants.size(), accepted.size(),
        n_hit_finger, n_hit_eebody, n_hit_arm5, n_hit_camera);

    rejected_pub_->publish(rejected_msg);
    accepted_pub_->publish(accepted_msg);
    publishDebugMarkers(rejected_debug, stamp);

    if (accepted.empty()) {
      response->success = false;
      response->message = "All variants rejected by rigid-block pre-filter.";
      RCLCPP_WARN(get_logger(), "[Prefilter] %s", response->message.c_str());
      return;
    }

    // ------------------------------------------------------------------
    // 5. 카메라 자세 점수로 정렬 (큰 값 = 카메라 위 향함, 우선)
    //    동점이면 입력 후보 순서 보존 (stable_sort), 같은 후보 내에서는
    //    원본(flipped=false) 이 flip 보다 약간 우선되도록 tiebreak.
    // ------------------------------------------------------------------
    std::stable_sort(accepted.begin(), accepted.end(),
        [](const Variant& a, const Variant& b) {
          if (a.cam_score != b.cam_score) {
            return a.cam_score > b.cam_score;  // 내림차순
          }
          if (a.orig_index != b.orig_index) {
            return a.orig_index < b.orig_index;  // 입력 순서 보존
          }
          return !a.flipped && b.flipped;        // 원본 우선
        });

    RCLCPP_INFO(get_logger(),
        "[Prefilter] Sorted by camera score. Top samples:");
    for (size_t k = 0; k < std::min<size_t>(5, accepted.size()); ++k) {
      const auto& v = accepted[k];
      RCLCPP_INFO(get_logger(),
          "  rank %zu: cand=%zu%s  cam_score=%+.4f",
          k + 1, v.orig_index + 1, v.flipped ? "*" : "", v.cam_score);
    }

    // ------------------------------------------------------------------
    // 6. 다음 노드(action_server) 호출
    // ------------------------------------------------------------------
    auto downstream_req = std::make_shared<ExecuteGraspSrv::Request>();
    downstream_req->arm_id = arm_id;
    downstream_req->grasp_candidates.header.stamp = this->get_clock()->now();
    downstream_req->grasp_candidates.header.frame_id = planning_frame_;
    downstream_req->grasp_candidates.poses.reserve(accepted.size());
    for (const auto& v : accepted) {
      downstream_req->grasp_candidates.poses.push_back(v.pose);
    }
    // [v2 디버깅] manager 가 전달한 request_id 를 그대로 forward
    //   (없으면 빈 문자열 → action_server 가 새로 생성)
    downstream_req->request_id = request->request_id;

    if (!downstream_client_->wait_for_service(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "Downstream service '" + downstream_service_name_ + "' not available.";
      RCLCPP_ERROR(get_logger(), "[Prefilter] %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(),
        "🚀 [Prefilter] Calling downstream '%s' with %zu candidates...",
        downstream_service_name_.c_str(), accepted.size());

    auto future = downstream_client_->async_send_request(downstream_req);
    auto status = future.wait_for(std::chrono::duration<double>(downstream_timeout_sec_));
    if (status != std::future_status::ready) {
      response->success = false;
      response->message = "Downstream call timed out after "
                        + std::to_string(downstream_timeout_sec_) + " s.";
      RCLCPP_ERROR(get_logger(), "[Prefilter] %s", response->message.c_str());
      return;
    }

    auto down_resp = future.get();
    response->success = down_resp->success;
    response->message = down_resp->message;

    auto t_end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    RCLCPP_INFO(get_logger(),
        "✅ [Prefilter] Done. downstream success=%s, elapsed=%.1f ms",
        response->success ? "TRUE" : "FALSE", elapsed_ms);
  }

  // ------------------------------------------------------------------
  // 멤버 변수
  // ------------------------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<ExecuteGraspSrv>::SharedPtr service_server_;
  rclcpp::Client<ExecuteGraspSrv>::SharedPtr downstream_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr rejected_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr accepted_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  moveit::core::RobotModelPtr robot_model_;
  std::string planning_frame_;

  // 카메라 캐시
  std::vector<Eigen::Vector3d> camera_corners_local_right_;
  std::vector<Eigen::Vector3d> camera_corners_local_left_;
  Eigen::Vector3d camera_local_dir_right_;
  Eigen::Vector3d camera_local_dir_left_;

  // 파라미터
  std::string input_service_name_;
  std::string downstream_service_name_;
  std::string planning_scene_service_;
  double downstream_timeout_sec_;
  double scene_timeout_sec_;
  bool enable_rigid_block_;
  bool enable_camera_check_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspPrefilterNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
