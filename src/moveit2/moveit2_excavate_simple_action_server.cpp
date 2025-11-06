#include "tms_if_for_opera/moveit2/moveit2_excavate_simple_action_server.hpp"

// #include <moveit_msgs/msg/constraints.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

using namespace tms_if_for_opera;

Moveit2ExcavateSimpleActionServer::Moveit2ExcavateSimpleActionServer(const rclcpp::NodeOptions& options)
  : Node("tms_if_for_opera_moveit2_excavate_simple", options)
{
  this->declare_parameter<std::string>("robot_description", "");
  this->get_parameter("robot_description", robot_description_);
  RCLCPP_INFO(this->get_logger(), "Robot description: %s", robot_description_.c_str());
  excavator_ik_.loadURDF(robot_description_);

  this->declare_parameter<std::string>("planning_group", "");
  this->get_parameter("planning_group", planning_group_);
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());

  std::string namespace_param = this->get_namespace();
  RCLCPP_INFO(this->get_logger(), "Node namespace: %s", namespace_param.c_str());

  this->declare_parameter<std::string>("collision_object_record_name", "");
  this->get_parameter("collision_object_record_name", collision_object_record_name_);
  RCLCPP_INFO(this->get_logger(), "Collision object record name: %s", collision_object_record_name_.c_str());

  this->declare_parameter<std::string>("collision_object_dump_record_name", "");
  std::string dump_record_names_str;
  this->get_parameter("collision_object_dump_record_name", dump_record_names_str);
  
  // コンマ区切りの文字列を配列に変換
  if (!dump_record_names_str.empty()) {
    std::stringstream ss(dump_record_names_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
      // 前後の空白を削除
      item.erase(0, item.find_first_not_of(" \t"));
      item.erase(item.find_last_not_of(" \t") + 1);
      if (!item.empty()) {
        collision_object_dump_record_name_.push_back(item);
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Collision object dump record name count: %zu", collision_object_dump_record_name_.size());

  /* Create server */
  RCLCPP_INFO(this->get_logger(), "Create server.");  // debug
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ExcavatorExcavateSimple>(
      this, "tms_rp_excavate_simple", std::bind(&Moveit2ExcavateSimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&Moveit2ExcavateSimpleActionServer::handle_cancel, this, _1),
      std::bind(&Moveit2ExcavateSimpleActionServer::handle_accepted, this, _1));
  /****/

  /* Setup movegroup interface */
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  move_group_node_ = rclcpp::Node::make_shared(std::string(this->get_name()) + "_move_group");

  // robot の状態監視のため
  executor_.add_node(move_group_node_);
  std::thread([this]() { executor_.spin(); }).detach();

  // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, planning_group_);
  move_group_options_ = std::make_shared<moveit::planning_interface::MoveGroupInterface::Options>(planning_group_, "robot_description", namespace_param);
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, *move_group_options_);
  
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Get robot info
  joint_names_ = move_group_->getJointNames();

  // For FK
  robot_state_ = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());

  // Init DB connection
  mongocxx::instance instance{};

  // For emg stop
  this->emg_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emg_stop", 10);
  
  // For waypoints visualization
  this->waypoints_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_markers", 10);
}

rclcpp_action::GoalResponse Moveit2ExcavateSimpleActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExcavatorExcavateSimple::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Moveit2ExcavateSimpleActionServer::handle_cancel(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Publishing EMG stop signal to Moveit2.");

  // 実機用非常停止
  std_msgs::msg::Bool msg;
  msg.data = true;
  this->emg_stop_publisher_->publish(msg);
  // move_group停止
  move_group_->stop();

  // auto result = std::make_shared<ExcavatorExcavateSimple::Result>();
  // result->error_code.val = 9999;
  // goal_handle->abort(result);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Moveit2ExcavateSimpleActionServer::handle_accepted(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "handle_accepted() start.");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&Moveit2ExcavateSimpleActionServer::execute, this, _1), goal_handle }.detach();
}

void Moveit2ExcavateSimpleActionServer::execute(const std::shared_ptr<GoalHandleExcavatorExcavateSimple> goal_handle)
{
  // Start to execute goal
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  move_group_->setGoalPositionTolerance(0.1);
  move_group_->setGoalOrientationTolerance(0.1);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setNumPlanningAttempts(100);
  move_group_->setPlanningTime(60.0);
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ExcavatorExcavateSimple::Feedback>();
  auto result = std::make_shared<ExcavatorExcavateSimple::Result>();

  // Function for error handling
  auto handle_error = [&](const std::string& message) {
    if (goal_handle->is_active())
    {
      result->error_code.val = 9999;
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), message.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal is not active");
    }
  };

  // Apply collision object
  apply_collision_objects_from_db(collision_object_record_name_);
  apply_collision_objects_mesh_from_db(collision_object_dump_record_name_);

  // Get link info
  link_names_ = move_group_->getLinkNames();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  // PlanningScene メッセージの作成
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;  // 差分更新
  // パディングを設定するための LinkPadding メッセージ
  moveit_msgs::msg::LinkPadding padding_msg;
  std::vector<std::string> padding_link_list = {"boom_link", "arm_link", "bucket_link", "bucket_end_link"};
  for (size_t i = 0; i < link_names_.size(); i++)
  {
      // リンク名が padding_link_list に含まれているかチェック
      if (std::find(padding_link_list.begin(), padding_link_list.end(), link_names_[i]) == padding_link_list.end())
      {
          continue;  // 含まれていなければスキップ
      }
      RCLCPP_INFO(this->get_logger(), "Link Name: %s", link_names_[i].c_str());
      // 新しく padding_msg を作成
      moveit_msgs::msg::LinkPadding padding_msg;
      padding_msg.link_name = link_names_[i];  // パディングを適用するリンク
      padding_msg.padding = 0.50;
      // パディングリストに追加
      planning_scene_msg.link_padding.push_back(padding_msg);
  }
  planning_scene_interface_.applyPlanningScene(planning_scene_msg);

  double radians = atan2(goal->position_with_angle.position.y, goal->position_with_angle.position.x);
  double swing_angle = radians;
  if (swing_angle < 0) swing_angle += 2 * M_PI;
  double offset = goal->position_with_angle.offset;

  // 入力パラメータを取得
  double x = goal->position_with_angle.position.x + offset * cos(swing_angle);
  double y = goal->position_with_angle.position.y + offset * sin(swing_angle);
  double z = goal->position_with_angle.position.z - 1.0;
  double theta_w = goal->position_with_angle.theta_w;

  RCLCPP_INFO(this->get_logger(), "Input: x=%.6f, y=%.6f, z=%.6f, theta_w=%.6f", x, y, z, theta_w);

  Pose target_pose;
  pose_converter_.convertToXYZQuaternion(x, y, z, theta_w, target_pose);
  geometry_msgs::msg::Pose moveit_pose;
  moveit_pose.position.x = target_pose.x;
  moveit_pose.position.y = target_pose.y;
  moveit_pose.position.z = target_pose.z;
  moveit_pose.orientation.x = target_pose.qx;
  moveit_pose.orientation.y = target_pose.qy;
  moveit_pose.orientation.z = target_pose.qz;
  moveit_pose.orientation.w = target_pose.qw;

  // Planning frameを確認
  std::string planning_frame = move_group_->getPlanningFrame();
  RCLCPP_INFO(this->get_logger(), "MoveIt Planning Frame: %s", planning_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Target pose in planning frame: x=%.3f, y=%.3f, z=%.3f", 
              moveit_pose.position.x, moveit_pose.position.y, moveit_pose.position.z);

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;

  // 最初の状態を取得
  moveit::core::RobotState start_state(*move_group_->getCurrentState());
  
  bool planning_success = true;
  const int max_planning_attempts = 10;  // 最大試行回数

  // 0. swing_angleだけ先に設定
  std::vector<double> swing_joint_values;
  start_state.copyJointGroupPositions(
    move_group_->getRobotModel()->getJointModelGroup(move_group_->getName()), 
    swing_joint_values);
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "swing_joint") {
      swing_joint_values[i] = swing_angle;
      break;
    }
  }
  
  moveit::planning_interface::MoveGroupInterface::Plan swing_plan;
  bool swing_plan_success = false;
  for (int attempt = 1; attempt <= max_planning_attempts && !swing_plan_success; attempt++) {
    RCLCPP_INFO(this->get_logger(), "Swing movement planning attempt %d/%d", attempt, max_planning_attempts);
    move_group_->setStartState(start_state);
    move_group_->setJointValueTarget(swing_joint_values);
    
    if (move_group_->plan(swing_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      swing_plan_success = true;
      plans.push_back(swing_plan);
      
      // 次の開始状態を更新
      start_state.setJointGroupPositions(
        move_group_->getRobotModel()->getJointModelGroup(move_group_->getName()), 
        swing_joint_values);
      start_state.update();
      
      RCLCPP_INFO(this->get_logger(), "Swing movement plan succeeded on attempt %d", attempt);
    } else if (attempt == max_planning_attempts) {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan swing movement after %d attempts", max_planning_attempts);
      planning_success = false;
    }
  }

  // 1. moveit_poseを中心とする円軌道をWaypointsで作成し、computeCartesianPathで動かす
  if (planning_success) {
    RCLCPP_INFO(this->get_logger(), "Creating circular trajectory around target pose");
    
    // 円の半径とウェイポイント数
    const double circle_radius = 1.5;  // 30cm
    const int num_waypoints = 16;  // 円周上の点の数
    
    // 入力パラメータを取得
    double center_x = goal->position_with_angle.position.x + offset * cos(swing_angle);
    double center_y = goal->position_with_angle.position.y + offset * sin(swing_angle);
    double center_z = goal->position_with_angle.position.z - 1.0;
    double theta_w = goal->position_with_angle.theta_w;
    
    RCLCPP_INFO(this->get_logger(), "Circle center (input params): x=%.3f, y=%.3f, z=%.3f, theta_w=%.3f", 
                center_x, center_y, center_z, theta_w);
    
    // base_linkからの水平角度を計算（掘削面の方向）
    double horizontal_angle = atan2(center_y, center_x);
    
    // 水平距離を計算
    double horizontal_distance = sqrt(center_x * center_x + center_y * center_y);
    
    RCLCPP_INFO(this->get_logger(), "Excavation plane: horizontal_angle=%.3f rad (%.1f deg), distance=%.3f m", 
                horizontal_angle, horizontal_angle * 180.0 / M_PI, horizontal_distance);
    
    // 円軌道のウェイポイントを入力パラメータ(x, y, z, theta_w)で生成
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // 下半分の半円を描く（垂直平面上）- 奥から手前へ
    for (int i = 0; i <= num_waypoints; i++) {
      // 下半分の円を逆方向に: 180度から0度（2πからπ）
      double angle = 2.0 * M_PI - (M_PI * i / num_waypoints);
      
      // 円軌道の計算（入力パラメータ座標系で）
      // - 水平方向（base_linkからの距離方向）の変化
      double radial_offset = circle_radius * cos(angle);
      // - 垂直方向（Z軸）の変化
      double vertical_offset = circle_radius * sin(angle);
      
      // 水平距離を変化させる
      double new_horizontal_distance = horizontal_distance + radial_offset;
      
      // 新しいx, y, z座標を計算（入力パラメータ座標系）
      double wp_x = new_horizontal_distance * cos(horizontal_angle);
      double wp_y = new_horizontal_distance * sin(horizontal_angle);
      double wp_z = center_z + vertical_offset;
      
      // theta_wを段階的に変化させる（0.5からπへ）
      double t = static_cast<double>(i) / num_waypoints;  // 0.0 から 1.0
      double wp_theta_w = 0.5 + t * (M_PI - 0.5);
      
      // pose_converterで変換
      Pose target_pose;
      pose_converter_.convertToXYZQuaternion(wp_x, wp_y, wp_z, wp_theta_w, target_pose);
      
      geometry_msgs::msg::Pose wp;
      wp.position.x = target_pose.x;
      wp.position.y = target_pose.y;
      wp.position.z = target_pose.z;
      wp.orientation.x = target_pose.qx;
      wp.orientation.y = target_pose.qy;
      wp.orientation.z = target_pose.qz;
      wp.orientation.w = target_pose.qw;
      
      waypoints.push_back(wp);
    }
    
    RCLCPP_INFO(this->get_logger(), "Generated %zu waypoints for semi-circular trajectory (lower half, back to front) in excavation plane", waypoints.size());
    RCLCPP_INFO(this->get_logger(), "theta_w range: 0.5 rad (%.1f deg) to π rad (%.1f deg)", 
                0.5 * 180.0 / M_PI, 180.0);
    
    // Planning frameを確認
    std::string planning_frame = move_group_->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "MoveIt Planning Frame: %s", planning_frame.c_str());
    if (!waypoints.empty()) {
      RCLCPP_INFO(this->get_logger(), "First waypoint (after conversion): x=%.3f, y=%.3f, z=%.3f", 
                  waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z);
      RCLCPP_INFO(this->get_logger(), "Last waypoint (after conversion): x=%.3f, y=%.3f, z=%.3f", 
                  waypoints.back().position.x, waypoints.back().position.y, waypoints.back().position.z);
    }
    
    // 1.1 最初のwaypointにsetPoseTargetで移動
    if (!waypoints.empty()) {
      RCLCPP_INFO(this->get_logger(), "Moving to first waypoint: x=%.3f, y=%.3f, z=%.3f", 
                  waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z);
      
      moveit::planning_interface::MoveGroupInterface::Plan first_waypoint_plan;
      bool first_waypoint_success = false;
      
      for (int attempt = 1; attempt <= max_planning_attempts && !first_waypoint_success; attempt++) {
        RCLCPP_INFO(this->get_logger(), "First waypoint planning attempt %d/%d", attempt, max_planning_attempts);
        move_group_->setStartState(start_state);
        move_group_->setPoseTarget(waypoints[0]);
        
        if (move_group_->plan(first_waypoint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          first_waypoint_success = true;
          plans.push_back(first_waypoint_plan);
          
          // planの終端姿勢を取得して次の開始状態にする
          const auto& trajectory = first_waypoint_plan.trajectory_.joint_trajectory;
          if (!trajectory.points.empty()) {
            const auto& last_point = trajectory.points.back();
            start_state.setJointGroupPositions(
              move_group_->getRobotModel()->getJointModelGroup(move_group_->getName()), 
              last_point.positions);
            start_state.update();
            
            RCLCPP_INFO(this->get_logger(), "First waypoint plan succeeded on attempt %d", attempt);
          }
        } else if (attempt == max_planning_attempts) {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan to first waypoint after %d attempts", max_planning_attempts);
          planning_success = false;
        }
      }
    }
    
    // 1.2 円軌道をcomputeCartesianPathで実行
    if (planning_success && !waypoints.empty()) {
      // computeCartesianPathでプランを作成
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;  // ジャンプ閾値
      const double eef_step = 0.01;  // エンドエフェクタのステップサイズ（1cm）
      
      move_group_->setStartState(start_state);
      double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      RCLCPP_INFO(this->get_logger(), "Cartesian path planning achieved %.2f%% of the path", fraction * 100.0);
      
      if (fraction > 0.9) {  // 90%以上のパスが計画できた場合
        // 軌道の速度を調整（joint_limitsを尊重）
        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
        rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
        
        // IterativeParabolicTimeParameterizationを使用（joint_limitsを尊重）
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool success = iptp.computeTimeStamps(rt, 1.0, 1.0);  // velocity_scaling=1.0, accel_scaling=1.0
        
        if (success) {
          rt.getRobotTrajectoryMsg(trajectory);
          RCLCPP_INFO(this->get_logger(), "Successfully applied time parameterization to speed up trajectory");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to apply time parameterization, using original trajectory");
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        plans.push_back(cartesian_plan);
        
        // 次の開始状態を更新
        if (!trajectory.joint_trajectory.points.empty()) {
          const auto& last_point = trajectory.joint_trajectory.points.back();
          start_state.setJointGroupPositions(
            move_group_->getRobotModel()->getJointModelGroup(move_group_->getName()), 
            last_point.positions);
          start_state.update();
          RCLCPP_INFO(this->get_logger(), "Circular Cartesian path plan succeeded");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Cartesian path planning only achieved %.2f%%, skipping circular motion", fraction * 100.0);
        planning_success = false;
      }
    }

    // ウェイポイントを可視化
    publish_waypoints_markers(waypoints, move_group_->getPlanningFrame());
  }

  if (!planning_success) {
    RCLCPP_WARN(this->get_logger(), "Aborting execution due to planning failure.");
    result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
    goal_handle->abort(result);
    return;
  }

  // 成功した場合、順番に実行
  for (const auto& plan : plans) {
    if (!move_group_->execute(plan)) {
      RCLCPP_ERROR(this->get_logger(), "Execution failed for one of the trajectories");
      result->error_code.val = static_cast<int>(rclcpp_action::ResultCode::ABORTED);
      goal_handle->abort(result);
      return;
    }
  }

  // If execution was successful, set the result of the action and mark it as succeeded.
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  result->error_code.val = 1;
  goal_handle->succeed(result);
}

void Moveit2ExcavateSimpleActionServer::apply_collision_objects_from_db(const std::string& record_name)
{
  // Load collision objects from DB
  // RCLCPP_INFO(this->get_logger(), "Loading collision objects from DB");

  mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
  mongocxx::database db = client["rostmsdb"];
  mongocxx::collection collection = db["parameter"];
  bsoncxx::builder::stream::document filter_builder;
  filter_builder << "record_name" << record_name;
  auto filter = filter_builder.view();
  auto result = collection.find_one(filter);

  if ((record_name != "") && !result)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB");
    return;
  }
  else if (record_name == "")
  {
    RCLCPP_INFO(this->get_logger(), "No collision objects to load");
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Succeeded to get collision objects from DB");
  }

  // Remove all collision objects
  // std::vector<std::string> object_ids = planning_scene_interface_.getKnownObjectNames();
  // planning_scene_interface_.removeCollisionObjects(object_ids);

  // Apply collision objects
  auto collision_objects = result->view()["collision_objects"].get_array().value;
  for (auto&& co : collision_objects)
  {
    moveit_msgs::msg::CollisionObject co_msg;
    co_msg.header.frame_id = move_group_->getPlanningFrame();
    co_msg.id = co["id"].get_utf8().value.to_string();

    // Get collision object type
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = co["primitive_type"].get_int32().value;
    for (auto dimension : co["dimensions"].get_array().value)
    {
      if (dimension.type() == bsoncxx::type::k_double)
      {
        primitive.dimensions.push_back(dimension.get_double().value);
      }
      else if (dimension.type() == bsoncxx::type::k_int32)
      {
        primitive.dimensions.push_back(static_cast<double>(dimension.get_int32().value));
      }
      else
      {
        throw std::runtime_error("Unsupported type");
      }
    }

    co_msg.primitives.push_back(primitive);

    // Get collision object pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = getDoubleValue(co["pose"]["position"]["x"]);
    pose.position.y = getDoubleValue(co["pose"]["position"]["y"]);
    pose.position.z = getDoubleValue(co["pose"]["position"]["z"]);
    pose.orientation.x = getDoubleValue(co["pose"]["orientation"]["x"]);
    pose.orientation.y = getDoubleValue(co["pose"]["orientation"]["y"]);
    pose.orientation.z = getDoubleValue(co["pose"]["orientation"]["z"]);
    pose.orientation.w = getDoubleValue(co["pose"]["orientation"]["w"]);

    co_msg.primitive_poses.push_back(pose);

    co_msg.operation = co_msg.ADD;

    planning_scene_interface_.applyCollisionObject(co_msg);
  }
}

void Moveit2ExcavateSimpleActionServer::apply_collision_objects_mesh_from_db(const std::vector<std::string>& record_names)
{
  for (const auto& record_name : record_names)
  {
    // Load collision objects from DB
    mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "record_name" << record_name;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);

    if ((record_name != "") && !result)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get collision objects from DB for record: %s", record_name.c_str());
      continue;
    }
    else if (record_name == "")
    {
      RCLCPP_INFO(this->get_logger(), "No collision objects to load");
      continue;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded to get collision objects from DB for record: %s", record_name.c_str());
    }

    auto collision_objects_dump = result->view();
    moveit_msgs::msg::CollisionObject co_dump_msg;
    co_dump_msg.header.frame_id = move_group_->getPlanningFrame();
    co_dump_msg.id = record_name + "_mesh"; // 一意のIDを付与

    // Apply collision objects
    auto mesh_binary = collision_objects_dump["data"].get_binary();
    std::string temp_mesh_path = "/tmp/temp_dump_mesh_" + record_name + ".dae";
    std::ofstream ofs(temp_mesh_path, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(mesh_binary.bytes), mesh_binary.size);
    ofs.close();
    shapes::Mesh *m = shapes::createMeshFromResource("file://" + temp_mesh_path);
    if (!m)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from temporary file for record: %s", record_name.c_str());
      continue;
    }
    shape_msgs::msg::Mesh mesh_msg;
    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(m, shape_msg);
    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    // Get collision object pose
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.position.x = collision_objects_dump["x"].get_double().value;
    mesh_pose.position.y = collision_objects_dump["y"].get_double().value;
    mesh_pose.position.z = collision_objects_dump["z"].get_double().value;
    mesh_pose.orientation.x = collision_objects_dump["qx"].get_double().value;
    mesh_pose.orientation.y = collision_objects_dump["qy"].get_double().value;
    mesh_pose.orientation.z = collision_objects_dump["qz"].get_double().value;
    mesh_pose.orientation.w = collision_objects_dump["qw"].get_double().value;

    // CollisionObjectにメッシュを追加
    co_dump_msg.meshes.push_back(mesh_msg);
    co_dump_msg.mesh_poses.push_back(mesh_pose);
    co_dump_msg.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface_.applyCollisionObject(co_dump_msg);
  }
}

double Moveit2ExcavateSimpleActionServer::getDoubleValue(const bsoncxx::document::element& element)
{
  if (element.type() == bsoncxx::type::k_double)
  {
    return element.get_double().value;
  }
  else if (element.type() == bsoncxx::type::k_int32)
  {
    return static_cast<double>(element.get_int32().value);
  }
  else
  {
    throw std::runtime_error("Unsupported type");
  }
}

void Moveit2ExcavateSimpleActionServer::publish_waypoints_markers(
    const std::vector<geometry_msgs::msg::Pose>& waypoints, 
    const std::string& frame_id)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 各ウェイポイントを球体マーカーとして表示
  for (size_t i = 0; i < waypoints.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = waypoints[i];
    
    marker.scale.x = 0.05;  // 5cm
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    // 色: 緑色
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 永続的に表示
    
    marker_array.markers.push_back(marker);
  }
  
  // ウェイポイント間を線で接続
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.frame_id = frame_id;
  line_strip.header.stamp = this->now();
  line_strip.ns = "waypoints_path";
  line_strip.id = waypoints.size();
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  
  line_strip.scale.x = 0.01;  // 線の太さ 1cm
  
  // 色: 青色
  line_strip.color.r = 0.0;
  line_strip.color.g = 0.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.8;
  
  line_strip.lifetime = rclcpp::Duration::from_seconds(0);
  
  // ウェイポイントの位置を線に追加
  for (const auto& wp : waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.position.x;
    p.y = wp.position.y;
    p.z = wp.position.z;
    line_strip.points.push_back(p);
  }
  
  marker_array.markers.push_back(line_strip);
  
  // マーカーを配信
  waypoints_marker_publisher_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published %zu waypoint markers", waypoints.size());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Moveit2ExcavateSimpleActionServer>());
  rclcpp::shutdown();
  return 0;
}