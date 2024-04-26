/*! \mainpage Main page for the floor robot demo
 *
 * \section Installation
 *
 * - cd ~/ariac_ws/src
 * - git clone https://github.com/zeidk/enpm663_summer2023.git -b lecture8
 * - cd ~/ariac_ws
 * - rosdep install --from-paths src -y -i --rosdistro galactic
 * - colcon build
 */

/*!
 *  \brief     Class for the floor robot.
 *  \details   The floor robot is capable of completing kitting tasks.
 *  \author    Zeid Kootbally
 *  \author    John Doe (add your teammate's name here)
 *  \version   0.1
 *  \date      July 2023
 *  \warning   Improper use can crash your application
 *  \copyright GNU Public License.
 */

#pragma once

// RCLCPP
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <moveit_msgs/msg/collision_object.hpp>
// Messages
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.h>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <robot_commander_msgs/srv/enter_tool_changer.hpp>
#include <robot_commander_msgs/srv/exit_tool_changer.hpp>
#include <robot_commander_msgs/srv/move_robot_to_table.hpp>
#include <robot_commander_msgs/srv/move_robot_to_tray.hpp>
#include <robot_commander_msgs/srv/move_tray_to_agv.hpp>
#include <robot_commander_msgs/srv/pick_part_from_bin.hpp>
#include <robot_commander_msgs/srv/place_part_on_tray.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
// TF2
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// KDL
#include <tf2_kdl/tf2_kdl.h>

#include <kdl/frames.hpp>
// C++
#include <unistd.h>

#include <cmath>

// #include <competitor_interfaces/msg/floor_robot_task.hpp>
// #include <competitor_interfaces/msg/completed_order.hpp>
// #include <competitor_msgs/msg/robots_status.hpp>

#include <geometry_msgs/msg/pose.hpp>

/**
 * @brief Class for the floor robot
 *
 * This class contains all the methods and attributes for the floor robot to
 * complete only kitting tasks
 *
 */
class FloorRobot : public rclcpp::Node {
  //-----------------------------//
  // Public methods
  //-----------------------------//
 public:
  /**
   * @brief Construct a new FloorRobot object
   *
   */
  FloorRobot();

  /**
   * @brief Destroy the FloorRobot object
   *
   */
  ~FloorRobot();
  //-----------------------------//

  /**
   * @brief Start the competition
   *
   * @return true  Successfully started the competition
   * @return false  Failed to start the competition
   */
  bool go_home();
  //-----------------------------//

  //-----------------------------//
  // Private attributes and methods
  //-----------------------------//
 private:
  //=========== START PYTHON - C++ ===========//
  /*
  The following snippets of code are used to send commands to the floor robot
  from Python. The idea is to implement the CCS in Python (read orders, find
  parts, find trays, etc.) and then send commands to the floor robot to do
  motion planning.
  */


  //! Service to move the robot to its home pose
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_robot_home_srv_;
  //! Service to move the robot to a table
  rclcpp::Service<robot_commander_msgs::srv::MoveRobotToTable>::SharedPtr
      move_robot_to_table_srv_;
  //! Service to move the robot very close to a tray (just before it is grasped)
  rclcpp::Service<robot_commander_msgs::srv::MoveRobotToTray>::SharedPtr
      move_robot_to_tray_srv_;
  //! Service to move the robot to an AGV after picking a tray
  rclcpp::Service<robot_commander_msgs::srv::MoveTrayToAGV>::SharedPtr
      move_tray_to_agv_srv_;
  //! Service to move the robot to an AGV after picking a tray
  rclcpp::Service<robot_commander_msgs::srv::PickPartFromBin>::SharedPtr
      pick_part_srv_;
  //! Service to move the robot to an AGV after picking a tray
  rclcpp::Service<robot_commander_msgs::srv::PlacePartOnTray>::SharedPtr
      place_part_srv_;
  //! Service to move the end effector inside a tool changer
  rclcpp::Service<robot_commander_msgs::srv::EnterToolChanger>::SharedPtr
      enter_tool_changer_srv_;
  //! Service to move the end effector outside a tool changer
  rclcpp::Service<robot_commander_msgs::srv::ExitToolChanger>::SharedPtr
      exit_tool_changer_srv_;

  /**
   * @brief Callback function for the service /commander/move_robot_home
   *
   * @param req_ Shared pointer to std_srvs::srv::Trigger::Request
   * @param res_ Shared pointer to std_srvs::srv::Trigger::Response
   */
  void move_robot_home_srv_cb(
      std_srvs::srv::Trigger::Request::SharedPtr req_,
      std_srvs::srv::Trigger::Response::SharedPtr res_);
  /**
   * @brief Callback function for the service /commander/move_robot_to_table
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::MoveToTable::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::MoveToTable::Response
   */
  void move_robot_to_table_srv_cb(
      robot_commander_msgs::srv::MoveRobotToTable::Request::SharedPtr req_,
      robot_commander_msgs::srv::MoveRobotToTable::Response::SharedPtr res_);

  /**
   * @brief Callback function for the service /commander/move_robot_to_tray
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::MoveToTray::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::MoveToTray::Response
   */
  void move_robot_to_tray_srv_cb(
      robot_commander_msgs::srv::MoveRobotToTray::Request::SharedPtr req_,
      robot_commander_msgs::srv::MoveRobotToTray::Response::SharedPtr res_);
  /**
   * @brief Callback function for the service
   * /competitor/floor_robot/move_tray_to_agv
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::MoveTrayToAGV::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::MoveTrayToAGV::Response
   */
  void move_tray_to_agv_srv_cb(
      robot_commander_msgs::srv::MoveTrayToAGV::Request::SharedPtr req_,
      robot_commander_msgs::srv::MoveTrayToAGV::Response::SharedPtr res_);

  /**
   * @brief Callback function for the service
   * /commander/pick_part_from_bin
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::PickPartFromBin::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::PickPartFromBin::Response
   */
  void pick_part_srv_cb(
      robot_commander_msgs::srv::PickPartFromBin::Request::SharedPtr req_,
      robot_commander_msgs::srv::PickPartFromBin::Response::SharedPtr res_);

  /**
   * @brief Callback function for the service
   * /commander/place_part_on_tray
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::PlacePartOnTray::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::PlacePartOnTray::Response
   */
  void place_part_srv_cb(
      robot_commander_msgs::srv::PlacePartOnTray::Request::SharedPtr req_,
      robot_commander_msgs::srv::PlacePartOnTray::Response::SharedPtr res_);

  /**
   * @brief  Callback function for the service /commander/enter_tool_changer
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::InToolChanger::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::InToolChanger::Response
   */
  void enter_tool_changer_srv_cb(
      robot_commander_msgs::srv::EnterToolChanger::Request::SharedPtr req_,
      robot_commander_msgs::srv::EnterToolChanger::Response::SharedPtr res_);

  /**
   * @brief Callback function for the service /commander/exit_tool_changer
   *
   * @param req_ Shared pointer to
   * robot_commander_msgs::srv::OutToolChanger::Request
   * @param res_ Shared pointer to
   * robot_commander_msgs::srv::OutToolChanger::Response
   */
  void exit_tool_changer_srv_cb(
      robot_commander_msgs::srv::ExitToolChanger::Request::SharedPtr req_,
      robot_commander_msgs::srv::ExitToolChanger::Response::SharedPtr res_);

  /**
   * @brief Provide motion to the floor robot to move its base to one of the two
   * tables.
   *
   * This method is called from FloorRobot::move_tray_to_agv_srv_cb
   * @param kts Either 1 or 2
   */
  bool move_robot_to_table(int kts);

  /**
   * @brief Provide motion to the floor robot to move the attached tray above an
   * AGV.
   *
   * @param agv_number
   * @return true  Motion successful
   * @return false  Motion failed
   */
  bool move_tray_to_agv(int agv_number);

  /**
   * @brief Provide motion to the floor robot to move the end effector just
   * above a tray.
   *
   * @param tray_id ID of the tray to pick up
   * @param tray_pose Pose of the tray to pick up
   * @return true Motion successful
   * @return false Motion failed
   */
  bool move_robot_to_tray(int tray_id,
                           const geometry_msgs::msg::Pose &tray_pose);


  /**
   * @brief  Move the robot to its home pose
   *
   * @return true Motion successful
   * @return false Motion failed
   */
  bool move_robot_home();

  /**
   * @brief Move the end effector inside a tool changer
   *
   * @param changing_station Name of the changing station
   * @param gripper_type Type of the gripper to change to
   * @return true Motion successful
   * @return false Motion failed
   */
  bool enter_tool_changer(std::string changing_station,
                           std::string gripper_type);

  /**
   * @brief Move the end effector outside a tool changer
   *
   * @param changing_station Name of the changing station
   * @param gripper_type Type of the gripper to change to
   * @return true Motion successful
   * @return false Motion failed
   */
  bool exit_tool_changer(std::string changing_station,
                          std::string gripper_type);

  //=========== END PYTHON - C++ ===========//

  /**
   * @brief Complete a single kitting task
   *
   * @param task  Kitting task to complete
   * @return true  Successfully completed the kitting task
   * @return false Failed to complete the kitting task
   */

  bool set_gripper_state(bool status);
  //-----------------------------//

  /**
   * @brief Change the gripper on the floor robot
   *
   * @param[in] changing_station Changing station to change the gripper
   * \parblock
   *  Possible values are "kts1" for kit tray station 1 and "kts2" for kit tray
   * station 2 \endparblock
   * @param[in] gripper_type Gripper type to change to
   * \parblock
   *  Possible values are "parts" for changing to parts gripper and "trays" for
   * changing to trays gripper \endparblock
   * @return true Successfully changed the gripper
   * @return false Failed to change the gripper
   */
  bool change_gripper(std::string changing_station, std::string gripper_type);
  //-----------------------------//

  /**
   * @brief Pick a tray from the kit tray station and place it on the AGV
   *
   * @param[in] tray_id ID of the tray to pick
   * \parblock
   *  Possible value is in the range [0,9]
   * \endparblock
   * @param[in] agv_num Number of the AGV to place the tray on
   * \parblock
   *  Possible value is in the range [1,4]
   * \endparblock
   * @return true Successfully picked and placed the tray
   * @return false Failed to pick and place the tray
   */
  bool pick_and_place_tray(int tray_id, int agv_num);
  //-----------------------------//

  /**
   * @brief Pick a part from the bin
   *
   * @param part_to_pick Part to pick
   * @return true  Successfully picked the part
   * @return false Failed to pick the part
   */
 bool pick_bin_part(ariac_msgs::msg::Part part_to_pick, const geometry_msgs::msg::Pose &pose);
  //-----------------------------//

  /**
   * @brief Place a part in a quadrant in the tray
   *
   * @param agv_num  AGV number
   * \parblock
   *  Possible value is in the range [1,4]
   * \endparblock
   * @param quadrant Quadrant in the tray
   * \parblock
   *  Possible value is in the range [1,4]
   * \endparblock
   * @return true Successfully placed the part in the tray
   * @return false Failed to place the part in the tray
   */
  bool place_part_in_tray(int agv_num, int quadrant);
  //-----------------------------//

  /**
   * @brief Move the floor robot to the target pose
   *
   * The target pose is set in the moveit_demo/config/floor_robot.yaml file
   * @return true Successfully moved to the target pose
   * @return false Failed to move to the target pose
   */
  bool move_to_target();
  //-----------------------------//

  /**
   * @brief Move the floor robot through waypoints
   *
   * @param waypoints  Waypoints to move through
   * @param vsf  Velocity scale factor
   * @param asf  Acceleration scale factor
   * @return true  Successfully moved through the waypoints
   * @return false  Failed to move through the waypoints
   */
  bool move_through_waypoints(std::vector<geometry_msgs::msg::Pose> waypoints,
                               double vsf, double asf);
  //-----------------------------//

  /**
   * @brief Wait for the gripper to attach the object
   *
   * @param timeout Timeout in seconds
   */
  void wait_for_attach_completion(double timeout);
  //-----------------------------//

  /**
   * @brief Set the orientation for the end effector of the robot
   *
   * @param yaw  Yaw angle in radians
   */
  geometry_msgs::msg::Quaternion set_robot_orientation(double yaw);
  //-----------------------------//

  /**
   * @brief Get the pose of a frame in the world frame
   *
   * @param frame_id Frame ID of the frame whose pose is to be found
   * @return geometry_msgs::msg::Pose  Pose of the frame in the world frame
   */
  geometry_msgs::msg::Pose get_pose_in_world_frame(std::string frame_id);
  //-----------------------------//

  /**
   * @brief Add a single model to the planning scene
   *
   * @param name  Name of the model
   * @param mesh_file  Mesh file of the model
   * @param model_pose  Pose of the model
   */
  void add_single_model_to_planning_scene(std::string name,
                                           std::string mesh_file,
                                           geometry_msgs::msg::Pose model_pose);
  //-----------------------------//

  /**
   * @brief Add static models to the planning scene
   *
   * Static models include the bins, tray tables, assembly stations, assembly
   * inserts, and the conveyor belt
   *
   */
  void add_models_to_planning_scene();
  //-----------------------------//

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  //! Current order being processed
  ariac_msgs::msg::Order current_order_;
  
  //! Move group interface for the floor robot
  moveit::planning_interface::MoveGroupInterfacePtr floor_robot_;
  //! Planning scene interface for the workcell
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  //! Trajectory processing for the floor robot
  /*!
  Generate the time-optimal trajectory along a given path within given bounds on
  accelerations and velocities.
  */
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
  //! Buffer used for TF2 transforms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_unique<tf2_ros::Buffer>(get_clock());
  //! TF2 listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  //! Subscriber for "/moveit_demo/floor_robot/go_home" topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moveit_demo_sub_;
  //! Subscriber for "/ariac/floor_robot_gripper_state" topic
  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr
      floor_gripper_state_sub_;
  // Orders List
  // competitor_interfaces::msg::FloorRobotTask current_order_;
  // std::vector<competitor_interfaces::msg::FloorRobotTask> orders_;

  //! State of the competition.
  /*!
      - IDLE=0 READY=1
      - STARTED=2
      - ORDER_ANNOUNCEMENTS_DONE=3
      - ENDED=4
  */
  unsigned int competition_state_;
  //! State of the gripper.
  /*!
      - enabled: True if the gripper is enabled.
      - attached: True if the gripper has an object attached.
      - type: Type of the gripper.
  */
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
  //! Part attached to the gripper.
  ariac_msgs::msg::Part floor_robot_attached_part_;
  //! Pose of the camera "kts1_camera" in the world frame
  /*!
  \note This attribute is set in the camera callback. You can hardcode it if you
  prefer.
  */
  geometry_msgs::msg::Pose kts1_camera_pose_;
  //! Pose of the camera "kts2_camera" in the world frame
  /*!
  \note This attribute is set in the camera callback. You can hardcode it if you
  prefer.
  */
  geometry_msgs::msg::Pose kts2_camera_pose_;
  //! Pose of the camera "left_bins_camera" in the world frame
  /*!
  \note This attribute is set in the camera callback. You can hardcode it if you
  prefer.
  */
  geometry_msgs::msg::Pose left_bins_camera_pose_;
  //! Pose of the camera "right_bins_camera" in the world frame
  /*!
  \note This attribute is set in the camera callback. You can hardcode it if you
  prefer.
  */
  geometry_msgs::msg::Pose right_bins_camera_pose_;
  //! Pose of trays found by "kts1_camera"
  std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
  //! Pose of trays found by "kts2_camera"
  std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;
  //! Pose of trays found by "left_bins_camera"
  std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
  //! Pose of trays found by "right_bins_camera"
  std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
  //! Callback group for the subscriptions
  rclcpp::CallbackGroup::SharedPtr subscription_cbg_;
  //! Specific callback group for the state of the gripper
  rclcpp::CallbackGroup::SharedPtr gripper_cbg_;
  //! Specific callback group for the tool changer client
  rclcpp::CallbackGroup::SharedPtr tool_changer_callback_group;
  //! Specific callback group for the gripper enable client
  rclcpp::CallbackGroup::SharedPtr gripper_enable_callback_group;
  //! Whether "kts1_camera" has received data or not
  bool kts1_camera_received_data = false;
  //! Whether "kts2_camera" has received data or not
  bool kts2_camera_received_data = false;
  //! Whether "left_bins_camera" has received data or not
  bool left_bins_camera_received_data = false;
  //! Whether "right_bins_camera" has received data or not
  bool right_bins_camera_received_data = false;
  //! Callback for "/moveit_demo/demo" topic
  void floor_robot_sub_cb(const std_msgs::msg::String::ConstSharedPtr msg);
  //! Callback for "/ariac/orders" topic
  void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);
  //! Callback for "/ariac/sensors/kts1_camera/image" topic
  void kts1_camera_cb(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  //! Callback for "/ariac/sensors/kts2_camera/image" topic
  void kts2_camera_cb(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  //! Callback for "/ariac/sensors/left_bins_camera/image" topic
  void left_bins_camera_cb(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  //! Callback for "/ariac/sensors/right_bins_camera/image" topic
  void right_bins_camera_cb(
      const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
  //! Callback for "/ariac/competition_state" topic
  void competition_state_cb(
      const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
  //! Callback for "/ariac/floor_robot_gripper_state" topic
  void floor_gripper_state_cb(
      const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
  //! Callback for "/ariac/agv1_status" topic
  void agv1_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
  //! Callback for "/ariac/agv2_status" topic
  void agv2_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
  //! Callback for "/ariac/agv3_status" topic
  void agv3_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
  //! Callback for "/ariac/agv4_status" topic
  void agv4_status_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);

  //! Client for "/ariac/perform_quality_check" service
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr
      quality_checker_;
  //! Client for "/ariac/floor_robot_change_gripper" service
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr
      floor_robot_tool_changer_;
  //! Client for "/ariac/floor_robot_enable_gripper" service
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr
      floor_robot_gripper_enable_;
  //! Thickness of the tray in meters
  /*! This is used to ensure the gripper does not collide with the tray when
   * placing a part in the tray */
  double kit_tray_thickness_ = 0.01;
  //! Distance between the tray and the gripper in meters.
  /*! This is used to place the gripper at a safe distance from the tray when
   * dropping a part in the tray */
  double drop_height_ = 0.002;
  //! Distance between the tray and the part in meters.
  /*! This is used to pick up a part */
  double pick_offset_ = 0.003;
  //! Mapping between part type constants and part type strings
  std::map<int, std::string> part_types_ = {
      {ariac_msgs::msg::Part::BATTERY, "battery"},
      {ariac_msgs::msg::Part::PUMP, "pump"},
      {ariac_msgs::msg::Part::REGULATOR, "regulator"},
      {ariac_msgs::msg::Part::SENSOR, "sensor"}};
  //! Mapping between part color constants and part color strings
  std::map<int, std::string> part_colors_ = {
      {ariac_msgs::msg::Part::RED, "red"},
      {ariac_msgs::msg::Part::BLUE, "blue"},
      {ariac_msgs::msg::Part::GREEN, "green"},
      {ariac_msgs::msg::Part::ORANGE, "orange"},
      {ariac_msgs::msg::Part::PURPLE, "purple"},
  };
  //! Mapping between part type constants and part heights
  std::map<int, double> part_heights_ = {
      {ariac_msgs::msg::Part::BATTERY, 0.04},
      {ariac_msgs::msg::Part::PUMP, 0.12},
      {ariac_msgs::msg::Part::REGULATOR, 0.07},
      {ariac_msgs::msg::Part::SENSOR, 0.07}};
  //! Mapping between quadrant type constants and offsets from the center of the
  //! tray
  std::map<int, std::pair<double, double>> quad_offsets_ = {
      {ariac_msgs::msg::KittingPart::QUADRANT1,
       std::pair<double, double>(-0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT2,
       std::pair<double, double>(0.08, 0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT3,
       std::pair<double, double>(-0.08, -0.12)},
      {ariac_msgs::msg::KittingPart::QUADRANT4,
       std::pair<double, double>(0.08, -0.12)},
  };
  //! Position of the linear actuator for different configurations
  std::map<std::string, double> rail_positions_ = {
      {"agv1", -4.5}, {"agv2", -1.2},   {"agv3", 1.2},
      {"agv4", 4.5},  {"left_bins", 3}, {"right_bins", -3}};
  //! Joint value targets for kit tray station 1
  std::map<std::string, double> floor_kts1_js_ = {
      {"linear_actuator_joint", 4.0},       {"floor_shoulder_pan_joint", 1.57},
      {"floor_shoulder_lift_joint", -1.57}, {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},       {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};
  //! Joint value targets for kit tray station 2
  std::map<std::string, double> floor_kts2_js_ = {
      {"linear_actuator_joint", -4.0},      {"floor_shoulder_pan_joint", -1.57},
      {"floor_shoulder_lift_joint", -1.57}, {"floor_elbow_joint", 1.57},
      {"floor_wrist_1_joint", -1.57},       {"floor_wrist_2_joint", -1.57},
      {"floor_wrist_3_joint", 0.0}};
  //! AGV locations for different AGVs.
  /*!
      The first value is the AGV number and the second value is the location of
     the AGV, the latter can be one of the following:
      - KITTING=0
      - ASSEMBLY_FRONT=1
      - ASSEMBLY_BACK=2
      - WAREHOUSE=3
      - UNKNOWN=99
  */
  std::map<int, int> agv_locations_ = {{1, -1}, {2, -1}, {3, -1}, {4, -1}};
};
