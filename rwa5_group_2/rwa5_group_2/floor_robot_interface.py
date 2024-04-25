import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from ariac_msgs.msg import Part
from ariac_msgs.msg import VacuumGripperState
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl

from geometry_msgs.msg import Pose


# Import custom ROS services
from robot_commander_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    PickPartFromBin,
    PlacePartOnTray,
)


class RobotCommanderInterface(Node):
    """
    Class for a robot commander node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    """

    def __init__(self):
        super().__init__("robot_commander")

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # The timer callback group is used to make the timer callback mutually exclusive
        # All timer callbacks are called in the same thread
        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        # The subscriber callback group is used to make the subscriber callback mutually exclusive
        # All subscriber callbacks are called in the same thread
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # ---- Subscribers ----

        # subscriber to the competition state
        # the state of the competition is used to decide when to:
        # - start the competition
        # - end the competition
        # - move the robot to the home position
        self.create_subscription(
            CompetitionState,
            "/ariac/competition_state",
            self._competition_state_cb,
            10,
            callback_group=subscriber_cb_group,
        )
        
        self._gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            "/ariac/floor_robot_gripper_state",
            self._gripper_state_cb,
            10,
            callback_group=subscriber_cb_group,
        )

        # ---- Clients ----

        # client to start the competition
        self._start_competition_cli = self.create_client(
            Trigger, "/ariac/start_competition"
        )

        # client to move the floor robot to the home position
        self._move_robot_home_cli = self.create_client(
            Trigger, "/commander/move_robot_home"
        )

        # client to move a robot to a table
        self._move_robot_to_table_cli = self.create_client(
            MoveRobotToTable, "/commander/move_robot_to_table"
        )

        # client to move a robot to a table
        self._move_robot_to_tray_cli = self.create_client(
            MoveRobotToTray, "/commander/move_robot_to_tray"
        )

        # client to move a tray to an agv
        self._move_tray_to_agv_cli = self.create_client(
            MoveTrayToAGV, "/commander/move_tray_to_agv"
        )
        
        # client to pick the part from bin
        self._pick_part_from_bin_cli = self.create_client(
            PickPartFromBin, "/commander/pick_part_from_bin"
        )
        
        # client to place the part on the tray
        self._place_part_on_tray_cli = self.create_client(
            PlacePartOnTray, "/commander/place_part_on_tray"
        )

        # client to move the end effector inside a tool changer
        self._enter_tool_changer_cli = self.create_client(
            EnterToolChanger, "/commander/enter_tool_changer"
        )

        # client to move the end effector outside a tool changer
        self._exit_tool_changer_cli = self.create_client(
            ExitToolChanger, "/commander/exit_tool_changer"
        )

        # client to activate/deactivate the vacuum gripper
        self._set_gripper_state_cli = self.create_client(
            VacuumGripperControl, "/ariac/floor_robot_enable_gripper"
        )

        # client to change the gripper type
        # the end effector must be inside the tool changer before calling this service
        self._change_gripper_cli = self.create_client(
            ChangeGripper, "/ariac/floor_robot_change_gripper"
        )

        # ---- Timer ----
        # Timer to trigger the robot actions
        # Every second the timer callback is called and based on the flags the robot actions are triggered
        self._main_timer = self.create_timer(
            1, self._main_timer_cb, callback_group=main_timer_cb_group
        )
        
        self.tray_flag = True
        self.part_flag = False
        self._grip_change = False
        
        # The following flags are used to ensure an action is not triggered multiple times
        self._moving_robot_home = False
        self._moving_robot_to_table = False
        self._entering_tool_changer = True
        self._changing_gripper = False
        self._exiting_tool_changer = False
        self._activating_gripper = False
        self._deactivating_gripper = False
        self._moving_robot_to_tray = False
        self._moving_robot_to_bin = False
        self._moving_tray_to_agv = False
        self._ending_demo = False

        # The following flags are used to trigger the next action
        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None
        self._moved_robot_home = False
        self._moved_robot_to_table = False
        self._entered_tool_changer = False
        self._changed_gripper = False
        self._exited_tool_changer = False
        self._activated_gripper = False
        self._deactivated_gripper = False
        self._moved_robot_to_tray = False
        self._picked_up_part_from_bin = False
        self._moved_tray_to_agv = False

        self.get_logger().info("Robot Commander Interface Node has been initialised.")

    def _competition_state_cb(self, msg: CompetitionState):
        """
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        """
        self._competition_state = msg.competition_state
    
    def _gripper_state_cb(self, msg: VacuumGripperState):
        """
        /ariac/floor_robot_gripper_state topic callback function

        Args:
            msg (VacuumGripperState): CompetitionState message
        """
        self._vacuum_gripper_state = msg
        
    def gripper_change(self,table,type,request):
        self.get_logger().info("In gripper change............")
        if not self._entering_tool_changer:
                    self._enter_tool_changer(table, type)
            
        if self._entered_tool_changer:
            if not self._changing_gripper:
                self._change_gripper(request)
        
        if self._changed_gripper:       
            if not self._exiting_tool_changer:
                self._exit_tool_changer(table, type)

    def _main_timer_cb(self):
        """
        Timer callback function to trigger the robot actions
        """

        self.get_logger().info('🚀 Robot action timer callback called')
        
        # Exit the callback if the kit is completed
        if self._kit_completed:
            self._main_timer.cancel()
            # check if the timer is cancelled
            if self._main_timer.is_canceled:
                self.get_logger().info("🏁 Timer cancelled")
            else:
                self.get_logger().info("🏁 Timer not cancelled")
        
        # # Start the competition
        # if not self._competition_started:
        #     if self._competition_state == CompetitionState.READY:
        #         self._start_competition()
        # move robot home
        # if self._competition_state == CompetitionState.STARTED:
        #     self.tray_flag = True
        if self.tray_flag:
            self.get_logger().info("In Tray Cycle............")
            if not self._moving_robot_home:
                self._move_robot_home()
            if self._moved_robot_home:
                if not self._moving_robot_to_table:
                    self._move_robot_to_table(MoveRobotToTable.Request.KTS1)
            if self._moved_robot_to_table:
                self.get_logger().info("Check gripper change")
                if str(self._vacuum_gripper_state.type) == "part_gripper":
                    self.get_logger().info("Gripper change needed")
                    self._grip_change = True
                    # self.gripper_change(self,"kts1","trays",ChangeGripper.Request.TRAY_GRIPPER)
                else:
                    self._grip_change = False
                    self._exited_tool_changer = True
            
            # if self._grip_change:
            self.get_logger().info(self._grip_change and (not self._entering_tool_changer))    
            if self._grip_change and (not self._entering_tool_changer):
                self._enter_tool_changer("kts1", "trays")
    
            if self._entered_tool_changer and self._grip_change:
                if not self._changing_gripper:
                    self._change_gripper(ChangeGripper.Request.TRAY_GRIPPER)
            
            if self._changed_gripper and self._grip_change:       
                self.get_logger().info("Exiting tool changer")
                if not self._exiting_tool_changer:
                    self._exit_tool_changer("kts1", "trays")
            if self._exited_tool_changer:
                if not self._activating_gripper:
                    self._activate_gripper()
            if self._activated_gripper:
            # Move the robot to the tray
            # TODO: Check whether the tray is picked up by using
            # a subscriber to /ariac/floor_robot_gripper_state
                if not self._moving_robot_to_tray:
                    # TODO: get the tray id and pose from the vision system
                    tray_id = MoveRobotToTray.Request.TRAY_ID1
                    tray_pose = Pose()
                    tray_pose.position.x = -0.870000
                    tray_pose.position.y = -5.840000
                    tray_pose.position.z = 0.734990
                    tray_pose.orientation.x = 0.0
                    tray_pose.orientation.y = 0.0
                    tray_pose.orientation.z = 1.0
                    tray_pose.orientation.w = 0.0
                    self._move_robot_to_tray(tray_id, tray_pose)
            
            if self._moved_robot_to_tray:
                # TODO: Check whether the tray is attached to the gripper before proceeding
                if not self._moving_tray_to_agv:
                    self._move_tray_to_agv(MoveTrayToAGV.Request.AGV1)
                    
            if self._moved_tray_to_agv:
                if not self._deactivating_gripper:
                    self._deactivate_gripper()
            if self._deactivated_gripper:
                self.tray_flag = False
                self.part_flag = True
                self._grip_change = False
                self._moving_robot_home = False
                self._moving_robot_to_table = False
                self._entering_tool_changer = True
                self._changing_gripper = False
                self._exiting_tool_changer = False
                self._activating_gripper = False
                self._deactivating_gripper = False
                self._moving_robot_to_tray = False
                self._moving_robot_to_bin = False
                self._moving_tray_to_agv = False
                self._ending_demo = False

                # The following flags are used to trigger the next action
                self._kit_completed = False
                self._competition_started = False
                self._competition_state = None
                self._moved_robot_home = False
                self._moved_robot_to_table = False
                self._entered_tool_changer = False
                self._changed_gripper = False
                self._exited_tool_changer = False
                self._activated_gripper = False
                self._deactivated_gripper = False
                self._moved_robot_to_tray = False
                self._picked_up_part_from_bin = False
                self._moved_tray_to_agv = False
        
        if self.part_flag:
            if not self._moving_robot_home:
                self._move_robot_home()
            if self._moved_robot_home:
                if not self._moving_robot_to_table:
                    self._move_robot_to_table(MoveRobotToTable.Request.KTS2)
            if self._moved_robot_to_table:
                if not self._vacuum_gripper_state.type == "part_gripper":
                    self.get_logger().info("Gripper change needed")
                    self._grip_change = True
                    # self.gripper_change(self,"kts1","trays",ChangeGripper.Request.TRAY_GRIPPER)
                else:
                    self._grip_change = False
            if (not self._entering_tool_changer) and self._grip_change:
                self._enter_tool_changer("kts2", "parts")
    
            if self._entered_tool_changer and self._grip_change:
                if not self._changing_gripper:
                    self._change_gripper(ChangeGripper.Request.PART_GRIPPER)
            
            if self._changed_gripper and self._grip_change:       
                if not self._exiting_tool_changer:
                    self.get_logger().info("Exiting tool changer")
                    self._exit_tool_changer("kts2", "parts")
            if not self._grip_change:
                if not self._activating_gripper:
                    self._activate_gripper()
            if self._activated_gripper:
                self._entering_tool_changer = True
                self._entered_tool_changer = False
                self._changed_gripper = False
            # Move the robot to the tray
            # TODO: Check whether the tray is picked up by using
            # a subscriber to /ariac/floor_robot_gripper_state
                if not self._moving_robot_to_bin:
                # TODO: get the tray id and pose from the vision system
                    part = Part()
                    part.color = PickPartFromBin.Request.ORANGE
                    part.type = PickPartFromBin.Request.BATTERY
                    part_pose = Pose()
                    part_pose.position.x = -2.080000
                    part_pose.position.y = 2.445000
                    part_pose.position.z = 0.722706
                    part_pose.orientation.x = 0.0
                    part_pose.orientation.y = 0.0
                    part_pose.orientation.z = 1.0
                    part_pose.orientation.w = 0.0
                    self._pick_part_from_bin(part, part_pose)
            
            if self._moving_robot_to_bin:
                # TODO: Check whether the tray is attached to the gripper before proceeding
                if not self._moving_tray_to_agv:
                    self._move_tray_to_agv(MoveTrayToAGV.Request.AGV1)
                    
            if self._moved_tray_to_agv:
                if not self._deactivating_gripper:
                    self._deactivate_gripper()
            # Deactivate gripper
            # TODO: Check whether the tray is not attached to the gripper
            # by using a subscriber to /ariac/floor_robot_gripper_state
            if self._deactivated_gripper:
                if not self._ending_demo:
                    self._move_robot_home(end_demo=True)

    def _start_competition(self):
        """
        Start the competition
        """

        self.get_logger().info("👉 Starting the competition...")

        while not self._start_competition_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = Trigger.Request()
        future = self._start_competition_cli.call_async(request)
        future.add_done_callback(self._start_competition_done_cb)

    def _start_competition_done_cb(self, future):
        """
        Client callback for the service /competitor/floor_robot/go_home

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._competition_started = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _move_robot_home(self, end_demo=False):
        """
        Move the floor robot to its home position
        """

        self.get_logger().info("👉 Moving robot home...")
        if end_demo:
            self._ending_demo = True
        else:
            self._moving_robot_home = True

        while not self._move_robot_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = Trigger.Request()
        future = self._move_robot_home_cli.call_async(request)
        future.add_done_callback(self._move_robot_home_done_cb)

    def _move_robot_home_done_cb(self, future):
        """
        Client callback for the service /competitor/floor_robot/go_home

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_home = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _move_robot_to_table(self, table_id):
        """
        Move the floor robot to a table

        Args:
            table_id (int): 1 for kts1 and 2 for kts2
        """

        self.get_logger().info("👉 Moving robot to changing station...")
        self._moving_robot_to_table = True
        while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = MoveRobotToTable.Request()
        request.kts = table_id
        future = self._move_robot_to_table_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_table_done_cb)

    def _move_robot_to_table_done_cb(self, future):
        """
        Client callback for the service /commander/move_robot_to_table

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_to_table = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _enter_tool_changer(self, station, gripper_type):
        """
        Move the end effector inside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        """
        self.get_logger().info("👉 Entering tool changer...")
        self._entering_tool_changer = True
        while not self._enter_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = EnterToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._enter_tool_changer_cli.call_async(request)
        future.add_done_callback(self._enter_tool_changer_done_cb)

    def _enter_tool_changer_done_cb(self, future):
        """
        Client callback for the service /commander/enter_tool_changer

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._entered_tool_changer = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _change_gripper(self, gripper_type):
        """
        Change the gripper

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        """
        self.get_logger().info("👉 Changing gripper...")
        self._changing_gripper = True
        while not self._change_gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = ChangeGripper.Request()
        request.gripper_type = gripper_type
        future = self._change_gripper_cli.call_async(request)
        future.add_done_callback(self._change_gripper_done_cb)

    def _change_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_change_gripper

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info("✅ Gripper changed")
            self._changed_gripper = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _exit_tool_changer(self, station, gripper_type):
        """
        Move the end effector outside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        """
        self.get_logger().info("👉 Exiting tool changer...")
        self._exiting_tool_changer = True
        while not self._exit_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = ExitToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._exit_tool_changer_cli.call_async(request)
        future.add_done_callback(self._exit_tool_changer_done_cb)

    def _exit_tool_changer_done_cb(self, future):
        """
        Client callback for the service /commander/exit_tool_changer

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._exited_tool_changer = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _activate_gripper(self):
        """
        Activate the gripper
        """
        self.get_logger().info("👉 Activating gripper...")
        self._activating_gripper = True
        
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = VacuumGripperControl.Request()
        request.enable = True
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._activate_gripper_done_cb)

    def _activate_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        """
        if future.result().success:
            self.get_logger().info("✅ Gripper activated")
            self._activated_gripper = True  
        else:
            self.get_logger().fatal("💀 Gripper not activated")

    def _deactivate_gripper(self):
        """
        Deactivate the gripper
        """
        self.get_logger().info("👉 Deactivating gripper...")
        self._deactivating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = VacuumGripperControl.Request()
        request.enable = False
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._deactivate_gripper_done_cb)

    def _deactivate_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_enable_gripper

        Args:
            future (Future): A future object
        """
        if future.result().success:
            self.get_logger().info("✅ Gripper deactivated")
            self._deactivated_gripper = True
        else:
            self.get_logger().fatal("💀 Gripper not deactivated")

    def _move_robot_to_tray(self, tray_id, tray_pose):
        """
        Move the floor robot to a tray to pick it up
        """
        self.get_logger().info("👉 Moving robot to tray...")
        self._moving_robot_to_tray = True
        
        while not self._move_robot_to_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available, waiting...")

        request = MoveRobotToTray.Request()
        request.tray_id = tray_id
        request.tray_pose_in_world = tray_pose
        future = self._move_robot_to_tray_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_tray_done_cb)

    def _move_robot_to_tray_done_cb(self, future):
        """
        Client callback for the service /commander/move_robot_to_tray

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_to_tray = True
        else:
            self.get_logger().fatal(f"💀 {message}")
    
    def _pick_part_from_bin(self, part, part_pose):
        """
        Move the floor robot to pick a part from bin
        """
        self.get_logger().info("👉 Moving robot to tray...")
        self._moving_robot_to_bin = True
        
        while not self._pick_part_from_bin_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available, waiting...")

        request = PickPartFromBin.Request()
        request.part = part
        request.uid = 0
        request.part_pose_in_world = part_pose
        future = self._pick_part_from_bin_cli.call_async(request)
        future.add_done_callback(self._pick_part_from_bin_done_cb)
    
    def _pick_part_from_bin_done_cb(self, future):
        """
        Client callback for the service /commander/pick_part_from_bin

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moving_robot_to_bin = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    # @brief Move the floor robot to its home position
    def _move_tray_to_agv(self, agv_number):
        
        self.get_logger().info("👉 Moving tray to AGV...")
        self._moving_tray_to_agv = True

        while not self._move_tray_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = MoveTrayToAGV.Request()
        request.agv_number = agv_number
        future = self._move_tray_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_tray_to_agv_done_cb)

    def _move_tray_to_agv_done_cb(self, future):
        """
        Client callback for the service /commander/move_tray_to_agv

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            if self.tray_flag:
                self._moved_tray_to_agv = True
            else:
                self._move_part_to_agv = True
        else:
            self.get_logger().fatal(f"💀 {message}")
