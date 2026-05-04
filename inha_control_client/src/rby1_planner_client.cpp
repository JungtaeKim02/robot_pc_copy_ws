#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <chrono>
#include <sstream>
#include <atomic>
#include <csignal>
#include <condition_variable>
#include <random>
#include <iomanip>

#include "inha_interfaces/srv/moveit_enable.hpp"
#include "inha_interfaces/srv/execute_grasp_srv.hpp"
#include "reachability_map.hpp"

// [v2 디버깅] UUID 형태 request_id 생성 헬퍼
namespace {
inline std::string generateRequestId() {
  // 단순 UUID v4-like (실제 RFC 4122 는 아니지만 충돌 방지로 충분)
  static thread_local std::mt19937_64 rng(std::random_device{}());
  std::uniform_int_distribution<uint64_t> dist;
  uint64_t a = dist(rng);
  uint64_t b = dist(rng);
  std::ostringstream oss;
  oss << std::hex << std::setfill('0')
      << std::setw(8) << ((a >> 32) & 0xFFFFFFFF) << "-"
      << std::setw(4) << ((a >> 16) & 0xFFFF) << "-"
      << std::setw(4) << (a & 0xFFFF) << "-"
      << std::setw(4) << ((b >> 48) & 0xFFFF) << "-"
      << std::setw(12) << (b & 0xFFFFFFFFFFFF);
  return oss.str();
}
}  // namespace

// TF2: link_torso_5 → link_torso_2 변환 (cutlery/anyplace pose 용)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// 전역 변수: 종료 요청 플래그
std::atomic<bool> g_shutdown_requested{false};

// 시그널 핸들러
void signal_handler(int signum)
{
  (void)signum;
  g_shutdown_requested.store(true);
  RCLCPP_WARN(rclcpp::get_logger("signal_handler"), "🛑 Shutdown requested (Ctrl+C)");
}

class GraspManager : public rclcpp::Node
{
public:
  using MoveitEnable = inha_interfaces::srv::MoveitEnable;
  using ExecuteGraspSrv = inha_interfaces::srv::ExecuteGraspSrv;

  GraspManager() : Node("rby1_grasp_manager")
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // 1. 리치맵 로드 (Torso 최적화 고려한 새로운 맵 적용)
    this->declare_parameter("left_map_path", "/home/nvidia/rby1_ws/src/rby1-ros2/rby1_control_client/reachability_maps/left_arm_reachability_torso.bin");
    this->declare_parameter("right_map_path", "/home/nvidia/rby1_ws/src/rby1-ros2/rby1_control_client/reachability_maps/right_arm_reachability_torso.bin");

    std::string left_path = this->get_parameter("left_map_path").as_string();
    std::string right_path = this->get_parameter("right_map_path").as_string();

    RCLCPP_INFO(get_logger(), "📥 Loading Reachability Maps...");
    if (left_map_.load(left_path)) RCLCPP_INFO(get_logger(), "✅ Left Map Loaded");
    else RCLCPP_WARN(get_logger(), "⚠️ Left Map Failed: %s", left_path.c_str());

    if (right_map_.load(right_path)) RCLCPP_INFO(get_logger(), "✅ Right Map Loaded");
    else RCLCPP_WARN(get_logger(), "⚠️ Right Map Failed: %s", right_path.c_str());

    // 2. Topic 구독 — GraspGen (PoseArray, link_torso_2 기준)
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/grasp_candidates", 10,
        std::bind(&GraspManager::pose_callback, this, std::placeholders::_1),
        sub_opt);
    
    RCLCPP_INFO(get_logger(), "📡 Subscribing to '/grasp_candidates' topic...");

    // 2-1. Topic 구독 — Cutlery (PoseStamped, link_torso_5 기준)
    cutlery_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cutlery/goal_pose", 10,
        std::bind(&GraspManager::cutlery_callback, this, std::placeholders::_1),
        sub_opt);
    RCLCPP_INFO(get_logger(), "📡 Subscribing to '/cutlery/goal_pose' topic...");

    // 2-2. Topic 구독 — Anyplace (PoseStamped, link_torso_5 기준)
    anyplace_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/anyplace/ee_goal_pose", 10,
        std::bind(&GraspManager::anyplace_callback, this, std::placeholders::_1),
        sub_opt);
    RCLCPP_INFO(get_logger(), "📡 Subscribing to '/anyplace/ee_goal_pose' topic...");
    
    // 퍼블리셔들
    filtered_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/filtered_grasp_candidates", 10);

    discarded_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/discarded_grasp_candidates", 10);

    RCLCPP_INFO(get_logger(), "✅ Manager Started. Visualizing filtered/discarded poses.");

    // 3. Service Server
    service_server_ = this->create_service<MoveitEnable>(
      "/manipulation/moveit/enable",
      std::bind(&GraspManager::handle_service_request, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_
    );

    // 4. Service Clients
    //    prefilter 경유 (graspgen 다수 후보용)
    client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_client_ = this->create_client<ExecuteGraspSrv>(
      "execute_grasp_service",
      rmw_qos_profile_services_default,
      client_callback_group_
    );

    //    action_server 직접 호출 (단일 pose용, prefilter 바이패스)
    direct_planning_client_ = this->create_client<ExecuteGraspSrv>(
      "execute_grasp_planning",
      rmw_qos_profile_services_default,
      client_callback_group_
    );

    // 5. TF2 — link_torso_5 → link_torso_2 변환용
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "✅ GraspManager initialized with multi-source support (graspgen/cutlery/anyplace).");
  }

private:
  rclcpp::Service<MoveitEnable>::SharedPtr service_server_;
  rclcpp::Client<ExecuteGraspSrv>::SharedPtr service_client_;
  rclcpp::Client<ExecuteGraspSrv>::SharedPtr direct_planning_client_;
  
  // GraspGen 구독 (PoseArray)
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseArray cached_candidates_;
  std::mutex cache_mutex_;
  bool candidates_received_ = false;

  // Cutlery 구독 (PoseStamped)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cutlery_sub_;
  geometry_msgs::msg::PoseStamped cached_cutlery_pose_;
  std::mutex cutlery_mutex_;
  bool cutlery_received_ = false;

  // Anyplace 구독 (PoseStamped)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr anyplace_sub_;
  geometry_msgs::msg::PoseStamped cached_anyplace_pose_;
  std::mutex anyplace_mutex_;
  bool anyplace_received_ = false;

  ReachabilityMap left_map_;
  ReachabilityMap right_map_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr filtered_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr discarded_pose_pub_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ================================================================
  // Topic Callbacks
  // ================================================================

  void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cached_candidates_ = *msg;
    candidates_received_ = true;
    RCLCPP_INFO(get_logger(), "📥 Received %zu grasp candidates (Frame: %s)", 
        msg->poses.size(), msg->header.frame_id.c_str());
  }

  void cutlery_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cutlery_mutex_);
    cached_cutlery_pose_ = *msg;
    cutlery_received_ = true;
    RCLCPP_DEBUG(get_logger(), "📥 Received cutlery pose (Frame: %s)",
        msg->header.frame_id.c_str());
  }

  void anyplace_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(anyplace_mutex_);
    cached_anyplace_pose_ = *msg;
    anyplace_received_ = true;
    RCLCPP_DEBUG(get_logger(), "📥 Received anyplace pose (Frame: %s)",
        msg->header.frame_id.c_str());
  }

  // ================================================================
  // TF 변환 헬퍼: PoseStamped → link_torso_2 기준으로 변환
  //   GraspGen 파이프라인은 link_torso_2 기준이므로 통일.
  //   cutlery/anyplace 는 link_torso_5 기준으로 들어오므로 TF 변환 필요.
  // ================================================================
  bool transformToTorso2(
      const geometry_msgs::msg::PoseStamped& input,
      geometry_msgs::msg::Pose& output_pose)
  {
    const std::string target_frame = "link_torso_2";

    if (input.header.frame_id == target_frame || input.header.frame_id.empty()) {
      output_pose = input.pose;
      return true;
    }

    try {
      geometry_msgs::msg::PoseStamped transformed;
      transformed = tf_buffer_->transform(input, target_frame, tf2::durationFromSec(1.0));
      output_pose = transformed.pose;
      RCLCPP_DEBUG(get_logger(), "[TF] '%s' → '%s' OK",
          input.header.frame_id.c_str(), target_frame.c_str());
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "[TF] '%s' → '%s' failed: %s",
          input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  // ================================================================
  // 서비스 핸들러 (통합 진입점)
  // ================================================================
  void handle_service_request(
    const std::shared_ptr<MoveitEnable::Request> request,
    std::shared_ptr<MoveitEnable::Response> response)
  {
    // pose_source 결정: 빈 문자열 → "graspgen" (하위 호환성)
    std::string pose_source = request->pose_source;
    if (pose_source.empty()) {
      pose_source = "graspgen";
    }

    RCLCPP_INFO(get_logger(), "📥 [Service Request] Arm ID: %s, Pose Source: %s",
        request->arm_id.c_str(), pose_source.c_str());

    // 종료 요청 체크
    if (g_shutdown_requested.load()) {
      response->success = false;
      response->message = "Service interrupted: Shutdown requested.";
      RCLCPP_WARN(get_logger(), "🛑 %s", response->message.c_str());
      return;
    }

    if (request->arm_id != "left" && request->arm_id != "right") {
      response->success = false;
      response->message = "Invalid arm_id. Use 'left' or 'right'.";
      RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
      return;
    }

    if (pose_source == "graspgen") {
      handle_graspgen(request->arm_id, response);
    } else if (pose_source == "cutlery" || pose_source == "anyplace") {
      handle_single_pose(request->arm_id, pose_source, response);
    } else {
      response->success = false;
      response->message = "Unknown pose_source: '" + pose_source + "'. Use 'graspgen', 'cutlery', or 'anyplace'.";
      RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
    }
  }

  // ================================================================
  // 기존 GraspGen 파이프라인 (PoseArray, 리치맵 → prefilter → action_server)
  // ================================================================
  void handle_graspgen(
    const std::string& arm_id,
    std::shared_ptr<MoveitEnable::Response> response)
  {
    geometry_msgs::msg::PoseArray candidates_to_process;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (!candidates_received_ || cached_candidates_.poses.empty()) {
        response->success = false;
        response->message = "No grasp candidates received from topic yet.";
        RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
        return;
      }
      candidates_to_process = cached_candidates_;
    }

    ReachabilityMap* current_map = (arm_id == "left") ? &left_map_ : &right_map_;
    if (!current_map->isLoaded()) {
        response->success = false;
        response->message = "Reachability Map not loaded.";
        RCLCPP_ERROR(get_logger(), "❌ %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "⚙️ [graspgen] Filtering candidates...");
    
    geometry_msgs::msg::PoseArray filtered_candidates;
    filtered_candidates.header = candidates_to_process.header; 
    filtered_candidates.header.stamp = this->now();

    geometry_msgs::msg::PoseArray discarded_candidates;
    discarded_candidates.header = candidates_to_process.header;
    discarded_candidates.header.stamp = filtered_candidates.header.stamp;

    std::vector<size_t> valid_indices;

    for (size_t i = 0; i < candidates_to_process.poses.size(); ++i) {
        const auto& pose = candidates_to_process.poses[i];
        
        if (current_map->isReachable(pose.position.x, pose.position.y, pose.position.z)) {
            filtered_candidates.poses.push_back(pose);
            valid_indices.push_back(i);
        } else {
            discarded_candidates.poses.push_back(pose);
        }
    }

    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        ss << valid_indices[i];
        if (i < valid_indices.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    RCLCPP_INFO(get_logger(), "📊 Filtered indices = %s", ss.str().c_str());

    filtered_pose_pub_->publish(filtered_candidates);
    discarded_pose_pub_->publish(discarded_candidates);

    size_t filtered_count = filtered_candidates.poses.size();
    size_t discarded_count = discarded_candidates.poses.size();
    
    RCLCPP_INFO(get_logger(), "🔍 Result: %zu Valid / %zu Discarded.", filtered_count, discarded_count);

    if (filtered_count == 0) {
        response->success = false;
        response->message = "All candidates filtered out (Unreachable).";
        RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
        return;
    }

    // 종료 요청 체크
    if (g_shutdown_requested.load()) {
      response->success = false;
      response->message = "Service interrupted: Shutdown requested before downstream call.";
      RCLCPP_WARN(get_logger(), "🛑 %s", response->message.c_str());
      return;
    }

    // prefilter 경유 하위 서비스 호출 (mode="" → default grasp)
    call_downstream(service_client_, "execute_grasp_service",
                    arm_id, filtered_candidates, "", response);
  }

  // ================================================================
  // 단일 Pose 파이프라인 (cutlery / anyplace)
  //   PoseStamped → TF → link_torso_2 → 리치맵 체크 →
  //   prefilter 바이패스 → action_server 직접 호출
  // ================================================================
  void handle_single_pose(
    const std::string& arm_id,
    const std::string& pose_source,
    std::shared_ptr<MoveitEnable::Response> response)
  {
    geometry_msgs::msg::PoseStamped source_pose;
    bool received = false;

    // 1. 캐시된 PoseStamped 가져오기
    if (pose_source == "cutlery") {
      std::lock_guard<std::mutex> lock(cutlery_mutex_);
      received = cutlery_received_;
      if (received) source_pose = cached_cutlery_pose_;
    } else if (pose_source == "anyplace") {
      std::lock_guard<std::mutex> lock(anyplace_mutex_);
      received = anyplace_received_;
      if (received) source_pose = cached_anyplace_pose_;
    }

    if (!received) {
      response->success = false;
      response->message = "No " + pose_source + " pose received from topic yet.";
      RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "⚙️ [%s] Processing single pose (frame: %s)...",
        pose_source.c_str(), source_pose.header.frame_id.c_str());

    // 2. TF 변환 → link_torso_2 기준 (리치맵이 link_torso_2 기준)
    geometry_msgs::msg::Pose pose_torso2;
    if (!transformToTorso2(source_pose, pose_torso2)) {
      response->success = false;
      response->message = "TF transform to link_torso_2 failed for " + pose_source + " pose.";
      RCLCPP_ERROR(get_logger(), "❌ %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "  [%s] pose in link_torso_2: pos=(%.3f, %.3f, %.3f)",
        pose_source.c_str(),
        pose_torso2.position.x, pose_torso2.position.y, pose_torso2.position.z);

    // 3. 리치맵 간단 체크 (단일 포인트)
    ReachabilityMap* current_map = (arm_id == "left") ? &left_map_ : &right_map_;
    if (current_map->isLoaded()) {
      if (!current_map->isReachable(pose_torso2.position.x,
                                     pose_torso2.position.y,
                                     pose_torso2.position.z)) {
        response->success = false;
        response->message = "[" + pose_source + "] Pose is outside reachability map.";
        RCLCPP_WARN(get_logger(), "❌ %s", response->message.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "  [%s] Reachability check: PASS ✅", pose_source.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "  [%s] Reachability map not loaded, skipping check.", pose_source.c_str());
    }

    // 4. PoseArray(1개)로 래핑 — action_server가 PoseArray를 기대
    //    frame_id는 source_pose 원본 frame 유지 (action_server/prefilter가
    //    planning frame으로 변환 처리함)
    geometry_msgs::msg::PoseArray single_pose_array;
    single_pose_array.header.stamp = this->now();
    single_pose_array.header.frame_id = source_pose.header.frame_id;
    single_pose_array.poses.push_back(source_pose.pose);

    // 종료 요청 체크
    if (g_shutdown_requested.load()) {
      response->success = false;
      response->message = "Service interrupted: Shutdown requested.";
      RCLCPP_WARN(get_logger(), "🛑 %s", response->message.c_str());
      return;
    }

    // 5. action_server 직접 호출 (prefilter 바이패스)
    //    단일 pose는 yaw flip / rigid-block / 카메라 점수 정렬이 불필요
    //    anyplace → mode="place" (Step2 cartesian 스킵, attach 스킵)
    //    cutlery  → mode="grasp" (기존 grasp 시퀀스)
    std::string mode = (pose_source == "anyplace") ? "place" : "grasp";
    RCLCPP_INFO(get_logger(), "🚀 [%s] Bypassing prefilter → direct to action_server (mode=%s)",
        pose_source.c_str(), mode.c_str());

    call_downstream(direct_planning_client_, "execute_grasp_planning",
                    arm_id, single_pose_array, mode, response);
  }

  // ================================================================
  // 공통: 하위 서비스 호출 (콜백 기반 비동기)
  // ================================================================
  void call_downstream(
    rclcpp::Client<ExecuteGraspSrv>::SharedPtr& client,
    const std::string& service_name,
    const std::string& arm_id,
    const geometry_msgs::msg::PoseArray& candidates,
    const std::string& mode,
    std::shared_ptr<MoveitEnable::Response> response)
  {
    if (!client->wait_for_service(2s)) {
        response->success = false;
        response->message = "Downstream service '" + service_name + "' unavailable.";
        RCLCPP_ERROR(get_logger(), "❌ %s", response->message.c_str());
        return;
    }

    auto client_req = std::make_shared<ExecuteGraspSrv::Request>();
    client_req->arm_id = arm_id;
    client_req->grasp_candidates = candidates;
    client_req->mode = mode;
    // [v2 디버깅] request_id 생성 및 전파 (downstream 노드에서 동일 ID 사용)
    client_req->request_id = generateRequestId();

    RCLCPP_INFO(get_logger(), "🚀 Forwarding %zu poses to '%s' (request_id=%s)...",
        candidates.poses.size(), service_name.c_str(), client_req->request_id.c_str());
    
    // 콜백 기반 비동기 처리
    std::mutex result_mutex;
    std::condition_variable result_cv;
    bool result_ready = false;
    std::shared_ptr<ExecuteGraspSrv::Response> result_response;
    bool result_error = false;
    std::string error_message;
    
    auto callback = [&](rclcpp::Client<ExecuteGraspSrv>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(result_mutex);
      try {
        result_response = future.get();
        RCLCPP_INFO(get_logger(), "📨 Callback received response!");
      } catch (const std::exception& e) {
        result_error = true;
        error_message = e.what();
        RCLCPP_ERROR(get_logger(), "❌ Callback exception: %s", e.what());
      }
      result_ready = true;
      result_cv.notify_one();
    };
    
    client->async_send_request(client_req, callback);
    
    // 응답 대기 (condition_variable 사용)
    {
      std::unique_lock<std::mutex> lock(result_mutex);
      while (!result_ready) {
        // 100ms마다 Ctrl+C 체크
        auto status = result_cv.wait_for(lock, 100ms);
        
        if (g_shutdown_requested.load()) {
          response->success = false;
          response->message = "Service interrupted: Shutdown requested during execution.";
          RCLCPP_WARN(get_logger(), "🛑 %s", response->message.c_str());
          return;
        }
        
        if (!rclcpp::ok()) {
          response->success = false;
          response->message = "ROS shutdown during execution.";
          RCLCPP_WARN(get_logger(), "🛑 %s", response->message.c_str());
          return;
        }
      }
    }

    // 결과 처리
    if (result_error) {
      response->success = false;
      response->message = "Exception getting result: " + error_message;
      RCLCPP_ERROR(get_logger(), "❌ %s", response->message.c_str());
      return;
    }
    
    if (!result_response) {
      response->success = false;
      response->message = "Null response from " + service_name + ".";
      RCLCPP_ERROR(get_logger(), "❌ %s", response->message.c_str());
      return;
    }
    
    response->success = result_response->success;
    response->message = result_response->message;

    if (response->success) {
        RCLCPP_INFO(get_logger(), "✅ Operation Successful: %s", response->message.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "⚠️ Operation Failed: %s", response->message.c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // 시그널 핸들러 등록
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  auto node = std::make_shared<GraspManager>();
  
  // MultiThreadedExecutor 사용 (여러 콜백 병렬 처리)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "🚀 GraspManager running with MultiThreadedExecutor (4 threads)...");
  
  // 별도 스레드에서 spin
  std::thread spin_thread([&executor]() {
    executor.spin();
  });
  
  // 메인 스레드에서 종료 대기
  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    std::this_thread::sleep_for(100ms);
  }
  
  RCLCPP_INFO(node->get_logger(), "🛑 Shutting down gracefully...");
  
  executor.cancel();
  
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  
  rclcpp::shutdown();
  return 0;
}
