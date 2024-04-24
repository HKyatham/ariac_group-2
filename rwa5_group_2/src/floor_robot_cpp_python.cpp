#include "floor_robot_cpp_python.hpp"
#include "utils.hpp"

FloorRobot::FloorRobot()
    : Node("floor_robot_demo_cpp_python"),
      node_(std::make_shared<rclcpp::Node>("example_group_node")),
      executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
      planning_scene_()
{
  auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
      "floor_robot",
      "robot_description");
  floor_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, mgi_options);

  if (floor_robot_->startStateMonitor())
  {
    RCLCPP_INFO(this->get_logger(), "Floor Robot State Monitor Started");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Floor Robot State Monitor Failed to Start");
  }

  // use upper joint velocity and acceleration limits
  floor_robot_->setMaxAccelerationScalingFactor(1.0);
  floor_robot_->setMaxVelocityScalingFactor(1.0);
  floor_robot_->setPlanningTime(10.0);
  floor_robot_->setNumPlanningAttempts(5);
  floor_robot_->allowReplanning(true);

  // callback groups
  rclcpp::SubscriptionOptions options;
  rclcpp::SubscriptionOptions gripper_options;
  subscription_cbg_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_cbg_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = subscription_cbg_;
  gripper_options.callback_group = gripper_cbg_;

  
  // subscription to /ariac/floor_robot_gripper/state
  floor_gripper_state_sub_ =
      this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
          "/ariac/floor_robot_gripper_state",
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
          std::bind(&FloorRobot::floor_gripper_state_cb, this,
                    std::placeholders::_1),
          gripper_options);
  // client to /ariac/perform_quality_check
  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>(
      "/ariac/perform_quality_check");
  // client to /ariac/floor_robot_change_gripper
  floor_robot_tool_changer_ =
      this->create_client<ariac_msgs::srv::ChangeGripper>(
          "/ariac/floor_robot_change_gripper");
  // client to /ariac/floor_robot_enable_gripper
  floor_robot_gripper_enable_ =
      this->create_client<ariac_msgs::srv::VacuumGripperControl>(
          "/ariac/floor_robot_enable_gripper");

  //---------------------------------//
  // Services
  //---------------------------------//

  // service to move the robot to home position
  move_robot_home_srv_ = create_service<std_srvs::srv::Trigger>(
      "/commander/move_robot_home",
      std::bind(&FloorRobot::move_robot_home_srv_cb, this,
                std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to move robot home created.");
  // service to move the robot to the table
  move_robot_to_table_srv_ =
      create_service<robot_commander_msgs::srv::MoveRobotToTable>(
          "/commander/move_robot_to_table",
          std::bind(&FloorRobot::move_robot_to_table_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to move robot to table created.");
  // service to move the robot to the tray
  move_robot_to_tray_srv_ =
      create_service<robot_commander_msgs::srv::MoveRobotToTray>(
          "/commander/move_robot_to_tray",
          std::bind(&FloorRobot::move_robot_to_tray_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to move robot to tray created.");
  // service to move the tray to the agv
  move_tray_to_agv_srv_ =
      create_service<robot_commander_msgs::srv::MoveTrayToAGV>(
          "/commander/move_tray_to_agv",
          std::bind(&FloorRobot::move_tray_to_agv_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to move robot tray to agv created.");
  // service to pick the part from the bin
  pick_part_srv_ =
      create_service<robot_commander_msgs::srv::PickPartFromBin>(
          "/commander/pick_part_from_bin",
          std::bind(&FloorRobot::pick_part_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to pick the part from the bin.");
  // service to place the part on tray
  place_part_srv_ =
      create_service<robot_commander_msgs::srv::PlacePartOnTray>(
          "/commander/place_part_on_tray",
          std::bind(&FloorRobot::place_part_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to place the part on tray.");
  // service to enter the tool changer
  enter_tool_changer_srv_ =
      create_service<robot_commander_msgs::srv::EnterToolChanger>(
          "/commander/enter_tool_changer",
          std::bind(&FloorRobot::enter_tool_changer_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to enter tool changer created.");
  // service to exit the tool changer
  exit_tool_changer_srv_ =
      create_service<robot_commander_msgs::srv::ExitToolChanger>(
          "/commander/exit_tool_changer",
          std::bind(&FloorRobot::exit_tool_changer_srv_cb, this,
                    std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service to exis tool changer created.");

  // add models to the planning scene
  add_models_to_planning_scene();
  RCLCPP_INFO(this->get_logger(), "Added models to planning scene.");
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]()
                                 { this->executor_->spin(); });

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
  RCLCPP_INFO(this->get_logger(), "Waiting for Service calls.");
}

//=============================================//
FloorRobot::~FloorRobot() { floor_robot_->~MoveGroupInterface(); }

//=============================================//
void FloorRobot::move_robot_home_srv_cb(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to move robot to home position");
  (void)req; // remove unused parameter warning

  if (go_home())
  {
    res->success = true;
    res->message = "Robot moved to home";
  }
  else
  {
    res->success = false;
    res->message = "Unable to move robot to home";
  }
}

//=============================================//
bool FloorRobot::move_robot_home()
{
  // Move floor robot to home joint state
  floor_robot_->setNamedTarget("home");
  return move_to_target();
}

//=============================================//
void FloorRobot::move_robot_to_table_srv_cb(
    robot_commander_msgs::srv::MoveRobotToTable::Request::SharedPtr req,
    robot_commander_msgs::srv::MoveRobotToTable::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to move robot to table");
  auto kts = req->kts;

  if (move_robot_to_table(kts))
  {
    res->success = true;
    res->message = "Robot moved to table";
  }
  else
  {
    res->success = false;
    res->message = "Unable to move robot to table";
  }
}

//=============================================//
bool FloorRobot::move_robot_to_table(int kts)
{
  if (kts == robot_commander_msgs::srv::MoveRobotToTable::Request::KTS1)
    floor_robot_->setJointValueTarget(floor_kts1_js_);
  else if (kts == robot_commander_msgs::srv::MoveRobotToTable::Request::KTS2)
    floor_robot_->setJointValueTarget(floor_kts2_js_);
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid table number");
    return false;
  }

  move_to_target();
  return true;
}

//=============================================//
void FloorRobot::move_robot_to_tray_srv_cb(
    robot_commander_msgs::srv::MoveRobotToTray::Request::SharedPtr req,
    robot_commander_msgs::srv::MoveRobotToTray::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to move robot to tray");
  auto tray_id = req->tray_id;
  auto tray_pose = req->tray_pose_in_world;

  if (move_robot_to_tray(tray_id, tray_pose))
  {
    RCLCPP_INFO(get_logger(), "Successfully moved robot to tray");
    res->success = true;
    res->message = "Robot moved to tray";
  }
  else
  {
    res->success = false;
    res->message = "Unable to move robot to tray";
  }
}

//=============================================//
bool FloorRobot::move_robot_to_tray(
    int tray_id, const geometry_msgs::msg::Pose &tray_pose)
{
  double tray_rotation = Utils::get_yaw_from_pose(tray_pose);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  waypoints.push_back(Utils::build_pose(tray_pose.position.x,
                                        tray_pose.position.y,
                                        tray_pose.position.z + pick_offset_,
                                        set_robot_orientation(tray_rotation)));

  if (!move_through_waypoints(waypoints, 0.3, 0.3))
  {
    RCLCPP_ERROR(get_logger(), "Unable to move robot above tray");
    return false;
  }

  // set_gripper_state(true);

  wait_for_attach_completion(5.0);

  if (floor_gripper_state_.attached)
  {
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    // Attach tray to robot in planning scene
    floor_robot_->attachObject(tray_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(Utils::build_pose(
        tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
        set_robot_orientation(tray_rotation)));

    if (!move_through_waypoints(waypoints, 0.2, 0.2))
    {
      RCLCPP_ERROR(get_logger(), "Unable to move up");
      return false;
    }
    return true;
  }

  return false;
}

//=============================================//
void FloorRobot::move_tray_to_agv_srv_cb(
    robot_commander_msgs::srv::MoveTrayToAGV::Request::SharedPtr req,
    robot_commander_msgs::srv::MoveTrayToAGV::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to move tray to agv");
  auto agv_number = req->agv_number;
  if (move_tray_to_agv(agv_number))
  {
    res->success = true;
    res->message = "Tray moved to AGV";
  }
  else
  {
    res->success = false;
    res->message = "Unable to move tray to AGV";
  }
}

//=============================================//
bool FloorRobot::move_tray_to_agv(int agv_number)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  floor_robot_->setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_number)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

  if (!move_to_target())
  {
    RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
    return false;
  }

  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_number) + "_tray");
  auto agv_rotation = Utils::get_yaw_from_pose(agv_tray_pose);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(agv_rotation)));

  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_,
      set_robot_orientation(agv_rotation)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
  {
    RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
    return false;
  }
  return true;
}

//=============================================//
void FloorRobot::enter_tool_changer_srv_cb(
    robot_commander_msgs::srv::EnterToolChanger::Request::SharedPtr req,
    robot_commander_msgs::srv::EnterToolChanger::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to enter tool changer");

  auto changing_station = req->changing_station;
  auto gripper_type = req->gripper_type;

  if (enter_tool_changer(changing_station, gripper_type))
  {
    res->success = true;
    res->message = "Entered tool changer";
  }
  else
  {
    res->success = false;
    res->message = "Unable to enter tool changer";
  }
}

//=============================================//
bool FloorRobot::enter_tool_changer(std::string changing_station,
                                    std::string gripper_type)
{
  usleep(10000);
  auto tc_pose = get_pose_in_world_frame(changing_station + "_tool_changer_" +
                                         gripper_type + "_frame");

  RCLCPP_INFO_STREAM(get_logger(),
                     "Tool changer pose: " << tc_pose.position.x << ", "
                                           << tc_pose.position.y << ", "
                                           << tc_pose.position.z);
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  return true;
}

//=============================================//
void FloorRobot::exit_tool_changer_srv_cb(
    robot_commander_msgs::srv::ExitToolChanger::Request::SharedPtr req,
    robot_commander_msgs::srv::ExitToolChanger::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to exit tool changer");

  auto changing_station = req->changing_station;
  auto gripper_type = req->gripper_type;

  if (exit_tool_changer(changing_station, gripper_type))
  {
    res->success = true;
    res->message = "Exited tool changer";
  }
  else
  {
    res->success = false;
    res->message = "Unable to exit tool changer";
  }
}

//=============================================//
bool FloorRobot::exit_tool_changer(std::string changing_station,
                                   std::string gripper_type)
{
  // Move gripper into tool changer
  auto tc_pose = get_pose_in_world_frame(changing_station + "_tool_changer_" +
                                         gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  return true;
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
void FloorRobot::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
  floor_gripper_state_ = *msg;
  // RCLCPP_INFO_STREAM(get_logger(), "Floor gripper state: " <<
  // floor_gripper_state_.attached);
}

geometry_msgs::msg::Pose FloorRobot::get_pose_in_world_frame(
    std::string frame_id)
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

  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("rwa5_group_2");
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

//=============================================//
void FloorRobot::add_models_to_planning_scene()
{
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
      {"bin1", std::pair<double, double>(-1.9, 3.375)},
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
  std::map<std::string, std::pair<double, double>> assembly_station_positions =
      {
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
    assembly_station_pose.orientation =
        Utils::get_quaternion_from_euler(0, 0, 0);

    add_single_model_to_planning_scene(station.first, "assembly_station.stl",
                                       assembly_station_pose);
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
    assembly_insert_pose.orientation =
        Utils::get_quaternion_from_euler(0, 0, 0);

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
geometry_msgs::msg::Quaternion FloorRobot::set_robot_orientation(
    double rotation)
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
  bool success = static_cast<bool>(floor_robot_->plan(plan));

  if (success)
  {
    return static_cast<bool>(floor_robot_->execute(plan));
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

  double path_fraction =
      floor_robot_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9)
  {
    RCLCPP_ERROR(get_logger(),
                 "Unable to generate trajectory through waypoints");
    return false;
  }

  // Retime trajectory
  robot_trajectory::RobotTrajectory rt(
      floor_robot_->getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_->getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_->execute(trajectory));
}

//=============================================//
void FloorRobot::wait_for_attach_completion(double timeout)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;

  while (!floor_gripper_state_.attached)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
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
  floor_robot_->setNamedTarget("home");
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
  auto request =
      std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
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
  auto tc_pose = get_pose_in_world_frame(changing_station + "_tool_changer_" +
                                         gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays")
  {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  }
  else if (gripper_type == "parts")
  {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto future = floor_robot_tool_changer_->async_send_request(request);

  future.wait();
  if (!future.get()->success)
  {
    RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation(0.0)));

  if (!move_through_waypoints(waypoints, 0.2, 0.1))
    return false;

  return true;
}

//=============================================//
bool FloorRobot::pick_and_place_tray(int tray_id, int agv_num) // need to add tray pose
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
    floor_robot_->setJointValueTarget(floor_kts1_js_);
  }
  else
  {
    floor_robot_->setJointValueTarget(floor_kts2_js_);
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
  waypoints.push_back(Utils::build_pose(tray_pose.position.x,
                                        tray_pose.position.y,
                                        tray_pose.position.z + pick_offset_,
                                        set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  set_gripper_state(true);

  wait_for_attach_completion(3.0);

  // Add kit tray to planning scene
  std::string tray_name = "kit_tray_" + std::to_string(tray_id);
  add_single_model_to_planning_scene(tray_name, "kit_tray.stl", tray_pose);
  floor_robot_->attachObject(tray_name);

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      tray_pose.position.x, tray_pose.position.y, tray_pose.position.z + 0.2,
      set_robot_orientation(tray_rotation)));
  move_through_waypoints(waypoints, 0.3, 0.3);

  floor_robot_->setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

  move_to_target();

  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");
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
  floor_robot_->detachObject(tray_name);

  // publish to robot state
  // LockAGVTray(agv_num);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      agv_tray_pose.position.x, agv_tray_pose.position.y,
      agv_tray_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.2, 0.1);

  return true;
}

void FloorRobot::pick_part_srv_cb(
    robot_commander_msgs::srv::MoveTrayToAGV::Request::SharedPtr req,
    robot_commander_msgs::srv::MoveTrayToAGV::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to pick the part");
  auto part = req->part;
  auto uid = req->uid;
  auto pose = req->part_pose_in_world;
  if (pick_bin_part(part, pose, uid))
  {
    res->success = true;
    res->message = "Picked the part from bin";
  }
  else
  {
    res->success = false;
    res->message = "Unable to pick the part from bin";
  }
}

//=============================================//
bool FloorRobot::pick_bin_part(ariac_msgs::msg::Part part_to_pick, const geometry_msgs::msg::Pose &pose, int uid)
{
  RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a "
                                       << part_colors_[part_to_pick.color]
                                       << " "
                                       << part_types_[part_to_pick.type]);

  // Check if part is in one of the bins
  geometry_msgs::msg::Pose part_pose = pose;
  std::string bin_side;

  // Check left bins
  if (part_pose.position.y < 0)
  {
    bin_side = "left_bins";
  }
  else
  {
    bin_side = "right_bins";
  }

  double part_rotation = Utils::get_yaw_from_pose(part_pose);

  // Change gripper at location closest to part
  if (floor_gripper_state_.type != "part_gripper")
  {
    std::string station;
    // Move floor robot to the corresponding kit tray table
    if (part_pose.position.y < 0)
    {
      station = "kts1";
      RCLCPP_INFO(get_logger(), "Moving the robot to table 1 for gripper change");
      floor_robot_->setJointValueTarget(floor_kts1_js_);
    }
    else
    {
      station = "kts2";
      RCLCPP_INFO(get_logger(), "Moving the robot to table 2 for gripper change");
      floor_robot_->setJointValueTarget(floor_kts2_js_);
    }

    
    move_to_target();

    change_gripper(station, "parts");
  }

  floor_robot_->setJointValueTarget("linear_actuator_joint",
                                   rail_positions_[bin_side]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
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

  wait_for_attach_completion(3.0);

  // Add part to planning scene
  std::string part_name =
      part_colors_[part_to_pick.color] + "_" + part_types_[part_to_pick.type];
  add_single_model_to_planning_scene(
      part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
  floor_robot_->attachObject(part_name);
  floor_robot_attached_part_ = part_to_pick;

  // Move up slightly
  waypoints.clear();
  waypoints.push_back(
      Utils::build_pose(part_pose.position.x, part_pose.position.y,
                        part_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  return true;
}

void FloorRobot::place_part_srv_cb(
    robot_commander_msgs::srv::MoveTrayToAGV::Request::SharedPtr req,
    robot_commander_msgs::srv::MoveTrayToAGV::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "Received request to pick the part");
  auto agv_number = req->agv_number;
  auto quadrant = req->quadrant;
  if (place_part_in_tray(agv_number, quadrant))
  {
    res->success = true;
    res->message = "Placed the part on tray";
  }
  else
  {
    res->success = false;
    res->message = "Unable to place part on tray";
  }
}

//=============================================//
bool FloorRobot::place_part_in_tray(int agv_num, int quadrant)
{
  if (!floor_gripper_state_.attached)
  {
    RCLCPP_ERROR(get_logger(), "No part attached");
    return false;
  }

  // Move to agv
  floor_robot_->setJointValueTarget(
      "linear_actuator_joint",
      rail_positions_["agv" + std::to_string(agv_num)]);
  floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
  move_to_target();

  // Determine target pose for part based on agv_tray pose
  auto agv_tray_pose =
      get_pose_in_world_frame("agv" + std::to_string(agv_num) + "_tray");

  auto part_drop_offset = Utils::build_pose(quad_offsets_[quadrant].first,
                                            quad_offsets_[quadrant].second, 0.0,
                                            geometry_msgs::msg::Quaternion());

  auto part_drop_pose = Utils::multiply_poses(agv_tray_pose, part_drop_offset);

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z +
          part_heights_[floor_robot_attached_part_.type] + drop_height_,
      set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.3, 0.3);

  // Drop part in quadrant
  set_gripper_state(false);

  std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" +
                          part_types_[floor_robot_attached_part_.type];
  floor_robot_->detachObject(part_name);

  waypoints.clear();
  waypoints.push_back(Utils::build_pose(
      part_drop_pose.position.x, part_drop_pose.position.y,
      part_drop_pose.position.z + 0.3, set_robot_orientation(0)));

  move_through_waypoints(waypoints, 0.2, 0.1);

  return true;
}