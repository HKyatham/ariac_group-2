#include "competition_control_system_interface.hpp"
#include "utils.hpp"

FloorRobot::FloorRobot()
    : Node("floor_robot_demo_cpp"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)),
                   "floor_robot"),
      planning_scene_()
  {
  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);
  floor_robot_.setPlanningTime(10.0);
  floor_robot_.setNumPlanningAttempts(5);
  floor_robot_.allowReplanning(true);
  // callback groups
  rclcpp::SubscriptionOptions options;
  rclcpp::SubscriptionOptions gripper_options;
  subscription_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = subscription_cbg_;
  gripper_options.callback_group = gripper_cbg_;

  // subscriber callback to /moveit_demo/floor_robot/go_home topic
  moveit_demo_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/moveit_demo/floor_robot/go_home", 10,
      std::bind(&FloorRobot::floor_robot_sub_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/orders
  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>(
      "/ariac/orders", 1,
      std::bind(&FloorRobot::orders_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/competition_state
  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
      "/ariac/competition_state", 1,
      std::bind(&FloorRobot::competition_state_cb, this,
                std::placeholders::_1),
      options);
  // subscription to /ariac/sensors/kts1_camera/image
  kts1_camera_sub_ = this->create_subscription<
      ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&FloorRobot::kts1_camera_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/sensors/kts2_camera/image
  kts2_camera_sub_ = this->create_subscription<
      ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&FloorRobot::kts2_camera_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/sensors/left_bins_camera/image
  left_bins_camera_sub_ = this->create_subscription<
      ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&FloorRobot::left_bins_camera_cb, this,
                std::placeholders::_1),
      options);
  // subscription to /ariac/sensors/right_bins_camera/image
  right_bins_camera_sub_ = this->create_subscription<
      ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
      std::bind(&FloorRobot::right_bins_camera_cb, this,
                std::placeholders::_1),
      options);
  // subscription to /ariac/floor_robot_gripper/state
  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
      "/ariac/floor_robot_gripper_state",
      rclcpp::QoS(rclcpp::KeepLast(1))
          .best_effort()
          .durability_volatile(),
      std::bind(&FloorRobot::floor_gripper_state_cb, this,
                std::placeholders::_1),
      gripper_options);
  // subscription to /ariac/agv1_status
  agv1_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv1_status", 10,
      std::bind(&FloorRobot::agv1_status_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/agv2_status
  agv2_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&FloorRobot::agv2_status_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/agv3_status
  agv3_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&FloorRobot::agv3_status_cb, this, std::placeholders::_1),
      options);
  // subscription to /ariac/agv4_status
  agv4_status_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&FloorRobot::agv4_status_cb, this, std::placeholders::_1),
      options);
  // client to /ariac/perform_quality_check
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>(
      "/ariac/perform_quality_check");
  // client to /ariac/floor_robot_change_gripper
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>(
      "/ariac/floor_robot_change_gripper");
  // client to /ariac/floor_robot_enable_gripper
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>(
      "/ariac/floor_robot_enable_gripper");

  // callback group for the timer
  main_timer_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  main_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&FloorRobot::main_timer_callback, this), main_timer_cbg_);

  // add models to the planning scene
  add_models_to_planning_scene();


  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  RCLCPP_INFO(this->get_logger(), "Waiting for Service calls.");
}

//=============================================//
FloorRobot::~FloorRobot() { floor_robot_.~MoveGroupInterface(); }

void FloorRobot::main_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Main timer callback");
  //Starting Competition
  if (!competition_started_)
  {
    start_competition();
    competition_started_ = true;
  }
  else
  {
    //Once competition is started, we move robot to home position once before beginning
    if (!moved_home_)
    {
      move_robot_home();
      moved_home_ = true;
    }
    // Completing the orders in the competition
    else if (!competition_ended_)
    {     
      competition_ended_= complete_orders();
      //Once competition is ended we are shutting down the node
      if (end_competition())
      {
        rclcpp::shutdown ();
      }

    }
    
  }
}
//=============================================//
bool FloorRobot::move_robot_home()
{
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  return move_to_target();
}


//=============================================//
bool FloorRobot::start_competition()
{
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY)
  {
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();
  RCLCPP_INFO(get_logger(), "Competition started");

  return result.get()->success;
}

//=============================================//
bool FloorRobot::end_competition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

//=============================================//
bool FloorRobot::lock_tray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

//=============================================//
bool FloorRobot::move_agv(int agv_num, int destination)
{
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result = client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

//=============================================//
void FloorRobot::agv1_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[1] = msg->location;
}

//=============================================//
void FloorRobot::agv2_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[2] = msg->location;
}

//=============================================//
void FloorRobot::agv3_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[3] = msg->location;
}

//=============================================//
void FloorRobot::agv4_status_cb(
    const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg)
{
  agv_locations_[4] = msg->location;
}

//=============================================//

void FloorRobot::orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg) {
  if (!msg) return;  
  order_object p_order;
  p_order.order = *msg;
  if(msg->priority){
    high_orders_.push_back(p_order);
  }
  else{
    low_orders_.push_back(p_order);
  }
}


//=============================================//
void FloorRobot::floor_robot_sub_cb(
    const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data == "go_home")
  {
    if (go_home())
    {
      RCLCPP_INFO(get_logger(), "Going home");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Unable to go home");
    }
  }
}

//=============================================//
void FloorRobot::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
  competition_state_ = msg->competition_state;
}

//=============================================//
void FloorRobot::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!kts1_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
    kts1_camera_received_data = true;
  }

  kts1_trays_ = msg->tray_poses;
  kts1_camera_pose_ = msg->sensor_pose;
}

//=============================================//
void FloorRobot::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!kts2_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
    kts2_camera_received_data = true;
  }

  kts2_trays_ = msg->tray_poses;
  kts2_camera_pose_ = msg->sensor_pose;
}

//=============================================//
void FloorRobot::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!left_bins_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from left bins camera");
    left_bins_camera_received_data = true;
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

//=============================================//
void FloorRobot::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!right_bins_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from right bins camera");
    right_bins_camera_received_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}
//==============================================//
void FloorRobot::agv1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!agv1_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from agv1 camera");
    agv1_camera_received_data = true;
  }

  agv1_parts_ = msg->part_poses;
  agv1_camera_pose_ = msg->sensor_pose;
}
//=============================================//
void FloorRobot::agv2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!agv2_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from agv2 camera");
    agv2_camera_received_data = true;
  }

  agv2_parts_ = msg->part_poses;
  agv2_camera_pose_ = msg->sensor_pose;
}
//=============================================//
void FloorRobot::agv3_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!agv3_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from agv3 camera");
    agv3_camera_received_data = true;
  }

  agv3_parts_ = msg->part_poses;
  agv3_camera_pose_ = msg->sensor_pose;
}
//=============================================//
void FloorRobot::agv4_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
  if (!agv4_camera_received_data)
  {
    RCLCPP_INFO(get_logger(), "Received data from agv4 camera");
    agv4_camera_received_data = true;
  }

  agv4_parts_ = msg->part_poses;
  agv4_camera_pose_ = msg->sensor_pose;
}
//=============================================//
void FloorRobot::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
  floor_gripper_state_ = *msg;
  
}

geometry_msgs::msg::Pose
FloorRobot::get_pose_in_world_frame(std::string frame_id)
{
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try
  {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

//=============================================//
void FloorRobot::add_single_model_to_planning_scene(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("final_group_2");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}
//==============================================//
void FloorRobot::remove_single_model_from_planning_scene(std::string name)
{
    std::vector<std::string> object_ids;
    object_ids.push_back(name);
    planning_scene_.removeCollisionObjects(object_ids);
}

//=============================================//
void FloorRobot::add_models_to_planning_scene()
{
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {{"bin1", std::pair<double, double>(-1.9, 3.375)},
                                                                    {"bin2", std::pair<double, double>(-1.9, 2.625)},
                                                                    {"bin3", std::pair<double, double>(-2.65, 2.625)},
                                                                    {"bin4", std::pair<double, double>(-2.65, 3.375)},
                                                                    {"bin5", std::pair<double, double>(-1.9, -3.375)},
                                                                    {"bin6", std::pair<double, double>(-1.9, -2.625)},
                                                                    {"bin7", std::pair<double, double>(-2.65, -2.625)},
                                                                    {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions)
  {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

    add_single_model_to_planning_scene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
      {"as1", std::pair<double, double>(-7.3, 3)},
      {"as2", std::pair<double, double>(-12.3, 3)},
      {"as3", std::pair<double, double>(-7.3, -3)},
      {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions)
  {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(
        station.first, "assembly_station.stl", assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions)
  {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(insert.first, "assembly_insert.stl",
                                       assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose = geometry_msgs::msg::Pose();
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("conveyor", "conveyor.stl",
                                     conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 3.14159);

  add_single_model_to_planning_scene("kts1_table", "kit_tray_table.stl",
                                     kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = Utils::get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene("kts2_table", "kit_tray_table.stl",
                                     kts2_table_pose);
}

//=============================================//
geometry_msgs::msg::Quaternion
FloorRobot::set_robot_orientation(double rotation)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);

  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q;
}

//=============================================//
bool FloorRobot::move_to_target()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success)
  {
    return static_cast<bool>(floor_robot_.execute(plan));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

//=============================================//
bool FloorRobot::move_through_waypoints(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(),
                 "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(
      floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}

//=============================================//
void FloorRobot::wait_for_attach_completion(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

  while (!floor_gripper_state_.attached)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.0002;
    waypoints.push_back(starting_pose);

    move_through_waypoints(waypoints, 0.1, 0.1);

    usleep(200);

    // if (floor_gripper_state_.attached)
    //     return;

    if (now() - start > rclcpp::Duration::from_seconds(timeout))
    {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
}

//=============================================//
bool FloorRobot::go_home()
{
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  return move_to_target();
}

//=============================================//
bool FloorRobot::set_gripper_state(bool enable)
{
  if (floor_gripper_state_.enabled == enable)
  {
    if (floor_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else
      RCLCPP_INFO(get_logger(), "Already disabled");

    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

//=============================================//
bool FloorRobot::change_gripper(std::string changing_station,
                                std::string gripper_type)
{
  // Move gripper into tool changer
  auto tc_pose = get_pose_in_world_frame(changing_station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(
      tc_pose.position.x, tc_pose.position.y, tc_pose.position.z + 0.4,
      set_robot_orientation(0.0)));

  waypoints.push_back(
      Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                        tc_pose.position.z, set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  }
  else if (gripper_type == "parts")
  {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto future = floor_robot_tool_changer_->async_send_request(request);

  future.wait();
  if (!future.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      tc_pose.position.x, tc_pose.position.y, tc_pose.position.z + 0.4,
      set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  return true;
}

//=============================================//
bool FloorRobot::pick_and_place_tray(int tray_id, int agv_num, std::string tr_nm)
{
  // Check if kit tray is on one of the two tables
  geometry_msgs::msg::Pose tray_pose;
  std::string station;
  bool found_tray = false;

  // Check table 1
  for (auto tray : kts1_trays_)
  {
    if (tray.id == tray_id)
    {
      station = "kts1";
      tray_pose = Utils::multiply_poses(kts1_camera_pose_, tray.pose);
      found_tray = true;
      break;
    }
  }
  // Check table 2
  if (!found_tray)
  {
    for (auto tray : kts2_trays_)
    {
      if (tray.id == tray_id)
      {
        station = "kts2";
        tray_pose = Utils::multiply_poses(kts2_camera_pose_, tray.pose);
        found_tray = true;
        break;
      }
    }
  }
  if (!found_tray)
    return false;

  double tray_rotation = Utils::get_yaw_from_pose(tray_pose);

  // Move floor robot to the corresponding kit tray table
  if (station == "kts1")
  {
    floor_robot_.setJointValueTarget(floor_kts1_js_);
  }
  else
  {
    floor_robot_.setJointValueTarget(floor_kts2_js_);
  }
  move_to_target();

  // Change gripper to tray gripper
  if (floor_gripper_state_.type != "tray_gripper")
  {
    change_gripper(station, "trays");
  }

  // Move to tray
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  waypoints.push_back(
      Utils::build_pose(tray_pose.position.x, tray_pose.position.y,
                        tray_pose.position.z + pick_offset_,
                        set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(20.0);

  // Add kit tray to planning scene
  std::string tray_name = tr_nm; //"kit_tray_" + std::to_string(tray_id);
  add_single_model_to_planning_scene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_.attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

  move_to_target();

  auto agv_tray_pose = get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");
  auto agv_rotation = Utils::get_yaw_from_pose(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(agv_rotation)));

  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_,
      set_robot_orientation(agv_rotation)));

  move_through_waypoints(waypoints, 0.2, 0.1);

  set_gripper_state(false);

  // object is detached in the planning scene
  floor_robot_.detachObject(tray_name);

  // publish to robot state
  // LockAGVTray(agv_num);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.2, 0.1);

  return true;
}
//========================================================//
bool FloorRobot::pick_bin_part(ariac_msgs::msg::Part part_to_pick , std::string part_nm)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a "
                                       << part_colors_[part_to_pick.color]
                                       << " "
                                       << part_types_[part_to_pick.type]);
  std::string part_name = part_nm ;
  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose;
  bool found_part = false;
  std::string bin_side;

  // Check left bins
  for (auto part : left_bins_parts_)
  {
    if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
    {
      part_pose = Utils::multiply_poses(left_bins_camera_pose_, part.pose);
      found_part = true;
      bin_side = "left_bins";
      break;
    }
  }
  // Check right bins
  if (!found_part)
  {
    for (auto part : right_bins_parts_)
    {
      if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
      {
        part_pose = Utils::multiply_poses(right_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "right_bins";
        break;
      }
    }
  }
  if (!found_part)
  {
    RCLCPP_ERROR(get_logger(), "Unable to locate part");
    return false;
  }

  double part_rotation = Utils::get_yaw_from_pose(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station;
    if (part_pose.position.y < 0)
    {
      station = "kts1";
    }
    else
    {
      station = "kts2";
    }

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
      floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
      floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    move_to_target();

    change_gripper(station, "parts");
  }

  floor_robot_.setJointValueTarget("linear_actuator_joint",
                                   rail_positions_[bin_side]);
  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.5,
      set_robot_orientation(part_rotation)));

  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y,
      part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_,
      set_robot_orientation(part_rotation)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(20.0);
  // Add part to planning scene
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_.attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;
  rclcpp::sleep_for(std::chrono::seconds(20));

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.3,
      set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);

   // Rotate to other direction
   if (bin_side == "left_bins")
    {
       floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 3.14); 
    }
    else
    {
       floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", -3.14);
    }
  move_to_target();
  if (!floor_gripper_state_.attached)
  {
    RCLCPP_ERROR(get_logger(), "No part attached");
    floor_robot_.detachObject(part_name);
    remove_single_model_from_planning_scene(part_name);
    set_gripper_state(false);
    part_attached = false;
    return false;
  }
  part_attached = true;
  return true;
}

//=============================================//
bool FloorRobot::place_part_in_tray(std::string id, int agv_num, int quadrant, std::string part_nm)
{ bool faulty_part = false;

  bool moved_to_agv = false ; 
  int moved_to_agv_counter = 0;
  std::string  part_name = part_nm;
  // Move to agv
  floor_robot_.setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  moved_to_agv = move_to_target();
  while (!moved_to_agv && moved_to_agv_counter < 20) {
    moved_to_agv_counter++;
    RCLCPP_INFO(get_logger(), "Moving floor robot to agv");
    moved_to_agv = move_to_target();
  }

  floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();
  if (!floor_gripper_state_.attached)
  {
    RCLCPP_ERROR(get_logger(), "No part attached");
    floor_robot_.detachObject(part_name);
    remove_single_model_from_planning_scene(part_name);
    set_gripper_state(false);
    part_attached = false;
    return false;
  }
  
  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose = get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = Utils::build_pose(
      quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
      geometry_msgs::msg::Quaternion());

  auto part_drop_pose = Utils::multiply_poses(agv_tray_pose, part_drop_offset);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
      set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();
  auto response = result.get();
  if (response->quadrant1.faulty_part || response->quadrant2.faulty_part || 
        response->quadrant3.faulty_part || response->quadrant4.faulty_part) {
        RCLCPP_INFO(get_logger(), "Faulty part detected");
        faulty_part = true;
    }
  if(faulty_part){
    RCLCPP_INFO(get_logger(), "Faulty part detected - initiating disposal");
    waypoints.clear();
    waypoints.push_back(Utils::build_pose(
    part_drop_pose.position.x, part_drop_pose.position.y,
    part_drop_pose.position.z + 0.3, set_robot_orientation(0)));
    move_through_waypoints(waypoints, 0.2, 0.1);
    floor_robot_.setJointValueTarget(disposal_bin_1_js_);   
    move_to_target();
    set_gripper_state(false);
    floor_robot_.detachObject(part_name);
    remove_single_model_from_planning_scene(part_name);
    waypoints.clear();
    return false;
  } else {
  // Drop part in quadrant
  set_gripper_state(false);
  floor_robot_.detachObject(part_name);
  RCLCPP_INFO(get_logger(), "part dropped on tray");
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));
  move_through_waypoints(waypoints, 0.2, 0.1);
  waypoints.clear();
  return true;
 }
}

// //=============================================//
bool FloorRobot::complete_orders() {
    while (low_orders_.empty() && high_orders_.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

  bool success = true; 
  auto current_order = low_orders_.front();
  if (low_order_parts_.empty()) {
      for (auto part : current_order.order.kitting_task.parts){
        low_order_parts_.push_back(part);
      }
    }
  
  if (!high_orders_.empty()){
    auto current_order = high_orders_.front();
  }

  while (!low_orders_.empty() || !high_orders_.empty() || competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false; 
      break;
    }

    if (!high_orders_.empty() && !current_order.order.priority){
      current_order = high_orders_.front();
    }
    if (current_order.order.type == ariac_msgs::msg::Order::KITTING) {
        complete_kitting_task(current_order);
    } else {
        RCLCPP_INFO(get_logger(), "Ignoring non-kitting tasks.");
    }
    if(current_order.submit){
      if(current_order.order.priority){
        high_orders_.erase(high_orders_.begin());
      }
      else{
        low_orders_.erase(low_orders_.begin());
      }
      if (high_orders_.empty()){
        if (low_orders_.empty()){
          break;
          }
          else{
            current_order = low_orders_.front();
            if (low_order_parts_.empty() && lower_order_flag == true) {
              for (auto part : current_order.order.kitting_task.parts){
                low_order_parts_.push_back(part);
              }
            }
          }
      }
      else{

        current_order = high_orders_.front();
      }
    }

    
  }

  if (success) {
    RCLCPP_INFO(get_logger(), "Completed all orders successfully.");
  } else {
    RCLCPP_ERROR(get_logger(), "Competition ended unexpectedly.");
  }

  return success;
}

//=============================================//
bool FloorRobot::submit_order(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;
  auto result = client->async_send_request(request);
  result.wait();
  return result.get()->success;
}
//=============================================//
bool FloorRobot::complete_kitting_task(order_object &order)
{
  if(!order.tray){
  go_home();
  planning_scene_tray_cnt_++;
  std::string tray_name = "kit_tray_" + std::to_string(order.order.kitting_task.tray_id)+"_"+std::to_string(planning_scene_tray_cnt_);

  pick_and_place_tray(order.order.kitting_task.tray_id, order.order.kitting_task.agv_number, tray_name);
  lock_tray(order.order.kitting_task.agv_number);
  RCLCPP_INFO(get_logger(), "Completed tray successfully.");
  order.tray = true;
  return true;
  }

  if(!order.part){
    if(!order.order.priority){
      lower_order_flag = false;
      for (auto it = low_order_parts_.begin(); it != low_order_parts_.end();)
      { 
        auto& kit_part = *it;
        bool part_picked = false;
        bool part_placed = false;
        planning_scene_part_cnt_++;
        std::string part_name = part_colors_[kit_part.part.color] + "_" + part_types_[kit_part.part.type]+"_"+std::to_string(planning_scene_part_cnt_);
        part_picked = pick_bin_part(kit_part.part ,part_name);
        if (part_picked){
          part_placed = place_part_in_tray(order.order.id,order.order.kitting_task.agv_number, kit_part.quadrant, part_name);
          if(part_placed){
            RCLCPP_INFO(get_logger(), "place part on tray returned true"); 
            it = low_order_parts_.erase(it);  
          }
          else {
            RCLCPP_INFO(get_logger(), "place part on tray returned false");
          }
        }
        else{
          RCLCPP_INFO(get_logger(), "part pick failed");
          if ((it != low_order_parts_.end()) && part_attached) {
                    ++it;
                }
          else if(!found_part){
            RCLCPP_INFO(get_logger(), "part not found going to next iteration");
            ++it;
          }    
        }
      }
    RCLCPP_INFO(get_logger(), "Outside  !order.part for loop, Number of low order parts: %zu", low_order_parts_.size());

    }
    else if(order.order.priority){
      for (auto it = order.order.kitting_task.parts.begin(); it != order.order.kitting_task.parts.end();) {
        auto& kit_part = *it;
        bool part_picked = false;
        bool part_placed = false;
        planning_scene_part_cnt_++;
        std::string part_name = part_colors_[kit_part.part.color] + "_" + part_types_[kit_part.part.type] + "_" + std::to_string(planning_scene_part_cnt_);
        part_picked = pick_bin_part(kit_part.part, part_name);
        if (part_picked) {
            part_placed = place_part_in_tray(order.order.id, order.order.kitting_task.agv_number, kit_part.quadrant, part_name);
            if (part_placed) {
                RCLCPP_INFO(get_logger(), "place part on tray returned true");   
                ++it;
            } else {
                RCLCPP_INFO(get_logger(), "place part on tray returned false");
                continue;
            }
        } else {
            RCLCPP_INFO(get_logger(), "part pick failed");
            if((it != order.order.kitting_task.parts.end()) && part_attached)
            {
            ++it;
            }
            else if(!found_part){
            RCLCPP_INFO(get_logger(), "part not found going to next iteration");
            ++it;
          } 
        }
    }
    }
    
  RCLCPP_INFO(get_logger(), "Completed parts");
  order.part = true;
  return true;
  }  

  if(!order.agv){
  if(!order.order.priority){
  RCLCPP_INFO(get_logger(), "Inside !order.agv, Number of low order parts: %zu", low_order_parts_.size());
  low_order_parts_.clear();
  lower_order_flag = true;
  }
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
  request->order_id = order.order.id;
  auto result = quality_checker_->async_send_request(request);
  result.wait();

  if (!result.get()->all_passed)
  {
    RCLCPP_INFO(get_logger(), "QC returned False. There is a possibility of an issue. Please take a look");
  }

  // move agv to destination
  move_agv(order.order.kitting_task.agv_number, order.order.kitting_task.destination);
  RCLCPP_INFO(get_logger(), "Completed agv successfully.");
  order.agv = true;
  return true;
  }
  if(!order.submit){
  int kitting_agv_num = -1;
  kitting_agv_num = order.order.kitting_task.agv_number;
  int agv_location = -1;
  while (agv_location != ariac_msgs::msg::AGVStatus::WAREHOUSE) {
    if (kitting_agv_num == 1)
      agv_location = agv_locations_[1];
    else if (kitting_agv_num == 2)
      agv_location = agv_locations_[2];
    else if (kitting_agv_num == 3)
      agv_location = agv_locations_[3];
    else if (kitting_agv_num == 4)
      agv_location = agv_locations_[4];
  }

  submit_order(order.order.id);
  RCLCPP_INFO(get_logger(), "Completed submit successfully.");
  order.submit = true;
  return true;
  }
  return true;
}
