import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32, Int32MultiArray
import time
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import json
import math
from geometry_msgs.msg import Pose
from ariac_msgs.msg import (
    Order,
    AdvancedLogicalCameraImage
)
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
)
from robot_commander_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    PickPartFromBin,
    PlacePartOnTray,
)
from ariac_msgs.srv import ChangeGripper, VacuumGripperControl


class ColorTypeMapper():
    '''
    Class for a Color Type Mapper.

    '''

    _part_colors = {
        PartMsg.RED: 'Red',
        PartMsg.BLUE: 'Blue',
        PartMsg.GREEN: 'Green',
        PartMsg.ORANGE: 'Orange',
        PartMsg.PURPLE: 'Purple',
    }
    '''Dictionary for converting Part color constants to strings'''

    _part_types = {
        PartMsg.BATTERY: 'Battery',
        PartMsg.PUMP: 'Pump',
        PartMsg.REGULATOR: 'Regulator',
        PartMsg.SENSOR: 'Sensor',
    }
    '''Dictionary for converting Part type constants to strings'''


class Part():
    def __init__(self,part):
        self.type = part.part.type
        self.color = part.part.color
        self.quadrant = part.quadrant

class Orders():
    def __init__(self, msg: Order):
        self.id = msg.id
        self.tray = {f'_{msg.kitting_task.tray_id}':set()}
        self.priority = msg.priority
        self.agv_number =  msg.kitting_task.agv_number
        self.destination = msg.kitting_task.destination
        self.parts = {}
        for part in msg.kitting_task.parts:
            p=Part(part)
            self.parts[f'_{p.color}_{p.type}'] = set()
                    
                    
                
            
class OrderSubInterface(Node):
    """A ROS2 OrderSubInterface node that listens to Order messages on a specified topic.

    This class creates a node that subscribes to the "/ariac/orders" topics, expecting messages of type `ariac_msgs/msg/Order`.
    It logs the part of received messages to the ROS2 logger and stores the message.

    Attributes:
        _orders_sub: Subscription object for receiving order messages from the topic '/ariac/orders'.
        _AGV_ID_pub: Publisher object for publishing AGV IDs to the topic 'fullfilled_agv_id'.
        _order_length_pub: Publisher object for publishing order lengths to the topic '/order_length'.
        _timer: Timer object for scheduling periodic callbacks.
        _clock: Clock object for accessing the current time.
        _counter: Counter variable.
        _h_counter: Helper counter variable.
        _h_prior: Boolean flag indicating the previous state of helper.
        _low_orders: List for storing low priority orders.
        _high_orders: List for storing high priority orders.
        _send_msg: Int32 message object.
        _send_order_length: Int32MultiArray message object.
        _camera_image: Placeholder for camera image data.
        _left_part_list: List for storing parts detected by the left camera.
        _right_part_list: List for storing parts detected by the right camera.
        _part_dic: Dictionary for storing information about parts.
        _part_parent_frame: Parent frame for broadcasting transform.
        _part_frame: Frame for broadcasting transform.
        tf_broadcaster: TransformBroadcaster object for broadcasting transforms.
        _tf_buffer: Buffer object for managing transforms.
        _tf_listener: TransformListener object for listening to transforms.
        trayCount: Dictionary for counting trays.
        partCount: Dictionary for counting parts.
        _dictionary: Dictionary for storing data.
        _tray_dict: Dictionary for storing tray data.
        right_bins_camera_sub: Subscription object for receiving camera images from the right bins camera.
        left_bins_camera_sub: Subscription object for receiving camera images from the left bins camera.
        kts1_camera_sub: Subscription object for receiving camera images from the kts1 camera.
        kts2_camera_sub: Subscription object for receiving camera images from the kts2 camera.
        _bridge: CvBridge object for converting images.
        _kts2_tray_data: Dictionary for storing kts2 tray data.
        _kts1_tray_data: Dictionary for storing kts1 tray data.
        _kts_tray_data_combined: Dictionary for storing combined kts tray data.
        _kts1_pos_data: Dictionary for storing kts1 position data.
        _kts2_pos_data: Dictionary for storing kts2 position data.
        _kts_pos_slot: Dictionary for storing kts position slots.
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """
    mutex_group1 = MutuallyExclusiveCallbackGroup()
    mutex_group2 = MutuallyExclusiveCallbackGroup()
    mutex_group3 = MutuallyExclusiveCallbackGroup()
    mutex_group4 = MutuallyExclusiveCallbackGroup()
    # mutex_group5 = MutuallyExclusiveCallbackGroup()
    
    def __init__(self):
        super().__init__('OrderSubInterface')
        #Subscribers
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10,callback_group= OrderSubInterface.mutex_group1)
        # self.Advcamera_sub = self.create_subscription(AdvancedLogicalCameraImageMsg,"/ariac/sensors/right_bins_camera/image",self.right_bins_camera_cb,qos_profile_sensor_data)
        #Publishers
        self._AGV_ID_pub = self.create_publisher(Int32, "fullfilled_agv_id", 10, callback_group= OrderSubInterface.mutex_group2)    
        self._order_length_pub = self.create_publisher(Int32MultiArray, "/order_length", 1, callback_group= OrderSubInterface.mutex_group2)
        #timers
        self._timer=self.create_timer(1,self.timer_cb, callback_group= OrderSubInterface.mutex_group2)    
        #Attributes
        self._clock = self.get_clock()
        self._counter=0
        self._h_counter=0
        self._h_prior=False
        self._low_orders = []   
        self._high_orders = []
        self._send_msg= Int32()
        self._send_order_length = Int32MultiArray()
        self._camera_image = None
        self._left_part_list=[]
        self._right_part_list=[]
        self._part_dic = {}
        # Information for broadcasting
        self._part_parent_frame = None
        self._part_frame = None
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.trayCount = {}
        self.partCount = {}
        self._part_frame_flag = False
        self._tray_frame_flag = False
        self._all_frame_flag = False
        self._lowCheck = None
        self._highCheck  = None
        # main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        # subscriber_cb_group = MutuallyExclusiveCallbackGroup()



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
        # self._set_gripper_state_cli = self.create_client(
        #     VacuumGripperControl, "/ariac/floor_robot_enable_gripper"
        # )

        # # client to change the gripper type
        # # the end effector must be inside the tool changer before calling this service
        # self._change_gripper_cli = self.create_client(
        #     ChangeGripper, "/ariac/floor_robot_change_gripper"
        # )

        # ---- Timer ----
        # Timer to trigger the robot actions
        # Every second the timer callback is called and based on the flags the robot actions are triggered
        # self._main_timer = self.create_timer(
        #     1, self._main_timer_cb, callback_group=main_timer_cb_group
        # )
        
        self.tray_flag = False
        self.part_flag = True
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

    

        colors = ['green', 'orange', 'blue', 'red', 'yellow', 'purple']
        components = ['sensor', 'battery', 'regulator', 'pump']
        colors = [0,1,2,3,4]
        components = [10,11,12,13]
        for color in colors:
            for component in components:
                attr_name = f"_{color}_{component}"
                setattr(self, attr_name, set())

        self._dictionary={}
        ids=[0,1,2,3,4,5,6,7,8,9]
        for id in ids:
                attr_name = f"_{id}"
                setattr(self, attr_name, set())
        self._tray_dict={}
        self.right_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,"/ariac/sensors/right_bins_camera/image",self._right_bins_camera_cb, qos_profile_sensor_data,callback_group= OrderSubInterface.mutex_group3)
        # self.right_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/right_bins_rgb_camera/rgb_image",self._right_bins_rgb_camera_cb,qos_profile_sensor_data)
        self.left_bins_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,"/ariac/sensors/left_bins_camera/image",self._left_bins_camera_cb, qos_profile_sensor_data,callback_group= OrderSubInterface.mutex_group3)
        # self.left_bins_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/left_bins_rgb_camera/rgb_image",self._left_bins_rgb_camera_cb,10)
        # self.kts1_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts1_rgb_camera/rgb_image",self._kts1_rgb_camera_cb,10)
        self.kts1_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,"/ariac/sensors/kts1_camera/image",self._kts1_camera_cb,qos_profile_sensor_data,callback_group= OrderSubInterface.mutex_group4)
        # self.kts2_rgb_camera_sub = self.create_subscription(Image,"/ariac/sensors/kts2_rgb_camera/rgb_image",self._kts2_rgb_camera_cb,10)
        self.kts2_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,"/ariac/sensors/kts2_camera/image",self._kts2_camera_cb,qos_profile_sensor_data,callback_group= OrderSubInterface.mutex_group4)
        self._bridge = CvBridge()
        self._kts2_tray_data ={}
        self._kts1_tray_data ={}
        self._kts_tray_data_combined = {}
        self._kts1_pos_data = {'tray_poses': []}
        self._kts2_pos_data = {'tray_poses': []}
        self._kts_pos_slot = {}
        
    # def quaternion_to_euler(self,quaternion):
    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw).

    #     Args:
    #         quaternion: Quaternion object representing the orientation.

    #     Returns:
    #         Tuple containing the euler angles (roll, pitch, yaw) in radians.

    #     Notes:
    #         Roll is rotation around x in radians (counterclockwise).
    #         Pitch is rotation around y in radians (counterclockwise).
    #         Yaw is rotation around z in radians (counterclockwise).
    #     """
        
    #     x,y,z,w=quaternion.x, quaternion.y, quaternion.z, quaternion.w
        

    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw)
    #     roll is rotation around x in radians (counterclockwise)
    #     pitch is rotation around y in radians (counterclockwise)
    #     yaw is rotation around z in radians (counterclockwise)
    #     """
        
    #     sinr_cosp = +2.0 * (w * x + y * z)
    #     cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)

    #     # pitch (y-axis rotation)
    #     sinp = +2.0 * (w * y - z * x)
    #     if (math.fabs(sinp) >= 1):
    #         pitch = math.copysign(math.M_PI / 2, sinp) # use 90 degrees if out of range
    #     else:
    #         pitch = math.asin(sinp)

    #     # yaw (z-axis rotation)
    #     siny_cosp = +2.0 * (w * z + x * y)
    #     cosy_cosp = +1.0 - 2.0 * (y * y + z * z)  
    #     yaw = math.atan2(siny_cosp, cosy_cosp)

    #     return roll, pitch, yaw

    def generate_transform(self, parent, child, pose):
        """
        Build a transform message and broadcast it.

        Args:
            parent (str): Parent frame.
            child (str): Child frame.
            pose (geometry_msgs.msg.Pose): Pose of the child frame with respect to the parent frame.

        Returns:
            geometry_msgs.msg.Pose: Transformed pose of the child frame.

        Notes:
            This method constructs a TransformStamped message with the given parent frame, child frame, and pose.
            It then broadcasts the transform and retrieves the transformed pose of the child frame.
        """
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = pose.pose.position.z
        transform_stamped.transform.rotation.x = pose.pose.orientation.x
        transform_stamped.transform.rotation.y = pose.pose.orientation.y
        transform_stamped.transform.rotation.z = pose.pose.orientation.z
        transform_stamped.transform.rotation.w = pose.pose.orientation.w

        # self._transforms.append(transform_stamped)
        # Send the transformation
        self.tf_broadcaster.sendTransform(transform_stamped)
        trans_pose=self._listener_cb(child)
        return trans_pose
    
    def _listener_cb(self,child):
        """
        Callback function for the listener timer.

        Args:
            child (str): Child frame for which transformation is being queried.

        Returns:
            list: Transformed pose of the child frame as [x, y, z, roll, pitch, yaw] if successful, None otherwise.

        Notes:
            This method queries the transform between the "world" frame and the specified child frame.
            It then computes the transformed pose in terms of translation (x, y, z) and orientation (roll, pitch, yaw).
            Returns None if the transform cannot be obtained.
        """
        try:
            if self._part_parent_frame is None:
                self.get_logger().warn("Part parent frame is not set.")
                return "No"          
            # Get the transform between frames
            t = self._tf_buffer.lookup_transform("world", child, rclpy.time.Time())
            transformed_x=t.transform.translation.x
            transformed_y=t.transform.translation.y
            transformed_z=t.transform.translation.z
            quat=t.transform.rotation
            Qx=quat.x
            Qy=quat.y
            Qz=quat.z
            Qw=quat.w
            
            # self.get_logger().info(f'DETECTED part  orientation: {quat}')

            # r,p,y=self.quaternion_to_euler(quat)
            # self.get_logger().info(f"Transform between world and {child}: \n")
                                #    + str(t.transform.translation)+ str(r)+ str(p)+ str(y))
            transformed_part_pose=[transformed_x,transformed_y,transformed_z,Qx,Qy,Qz,Qw]
            if self._part_frame_flag  and self._tray_frame_flag:
                self._all_frame_flag = True
            return transformed_part_pose
        
        except TransformException as ex:
            self.get_logger().fatal(
                f"Could not get transform between world and {child}: {str(ex)}"
            )
            return "No"

        
    
    def _right_bins_camera_cb(self,msg):
        """
        Callback function for processing messages from the right bins camera.

        Args:
            msg (AdvancedLogicalCameraImage): Message containing information about detected parts.

        Returns:
            None

        Notes:
            This method processes messages received from the right bins camera.
            It extracts information about detected parts and calculates their positions and orientations.
            The transformed coordinates and orientations are then stored in the appropriate data structures.
        """
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            else:
                # if self._right_bin_parts != len(msg._part_poses):
                    for i, part_pose in enumerate(msg._part_poses):
                        # self.get_logger().debug(f'Right bin deteected part numb: {i+1}: ')
                        color=part_pose.part.color
                        comp=part_pose.part.type
                        self._part_parent_frame = "right_bins_camera_frame"
                        self._part_frame = f"right_bin_part_{i+1}_frame"
                        # self._right_broadcaster_part_pose(pose)
                        trans_coords =self.generate_transform(self._part_parent_frame, self._part_frame, part_pose)
                        if trans_coords == "No":
                            self._part_frame_flag = False
                        else:
                            self._part_frame_flag = True
                            X=round(trans_coords[0],6)
                            Y=round(trans_coords[1],6)
                            Z=round(trans_coords[2],6)
                            oX=round(trans_coords[3],6)
                            oY=round(trans_coords[4],6)
                            oZ=round(trans_coords[5],6)
                            oW=round(trans_coords[6],6)
                            
                            # oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            # oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            # oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # oW=0.0 if math.isclose(round())
                            # self.get_logger().info(f'Part {i+1} Location: Color :{color}, Typee :{comp},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            colors = [0,1,2,3,4]
                            components = [10,11,12,13]
                            ort_var=(X,Y,Z,oX,oY,oZ,oW)
                            for colorr in colors:
                                for component in components:
                                    if color == colorr and comp == component:
                                        attr_name = f"_{colorr}_{component}"
                                        if hasattr(self, attr_name):
                                            target_list = getattr(self, attr_name)
                                            if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) \
                                                        and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[3],list(target_list)[i][3], abs_tol=0.01))\
                                                        and (math.isclose(ort_var[4],list(target_list)[i][4], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[5],list(target_list)[i][5], abs_tol=0.01)) \
                                                        and  (math.isclose(ort_var[6],list(target_list)[i][6], abs_tol=0.01)):
                                                        ispresent=True
                                                if ispresent==False:
                                                    target_list.add(ort_var)
                                                    self._dictionary.update({attr_name:target_list})
                                                # self.get_logger().debug(f"Updated {attr_name}: {ort_var}")
                    self.get_logger().debug(f"Added parts dict from right cam cb: {self._dictionary}") 
                    # self._right_bin_parts = len(msg._part_poses)
                    # self.get_logger().info(f"No. of parts in right bin: {self._right_bin_parts}") 


                                

        except Exception as e:
            self.get_logger().error('Right ALC failed to process the part poses: %r' % (e,))
        
   
    def _left_bins_camera_cb(self,msg):
        """
        Callback function for processing messages from the left bins camera.

        Args:
            msg (AdvancedLogicalCameraImage): Message containing information about detected parts.

        Returns:
            None

        Notes:
            This method processes messages received from the left bins camera.
            It extracts information about detected parts and calculates their positions and orientations.
            The transformed coordinates and orientations are then stored in the appropriate data structures.
        """
        try:
            if len(msg._part_poses) == 0:
                self.get_logger().info('NO Part DETECTED')
            else: 
                # if self._left_bin_parts != len(msg._part_poses):
                        
                    for i, part_pose in enumerate(msg._part_poses):
                        # self.get_logger().info(f'DETECTED part {i+1}: ')
                        color=part_pose.part.color
                        comp=part_pose.part.type
                        self._part_parent_frame = "left_bins_camera_frame"
                        self._part_frame = f"left_bin_part_{i+1}_frame"
                        # self._right_broadcaster_part_pose(pose)
                        trans_coords =self.generate_transform(self._part_parent_frame, self._part_frame, part_pose)
                        if trans_coords == "No":
                            self._part_frame_flag = False
                        else:
                            self._part_frame_flag = True
                            X=round(trans_coords[0],6)
                            Y=round(trans_coords[1],6)
                            Z=round(trans_coords[2],6)
                            oX=round(trans_coords[3],6)
                            oY=round(trans_coords[4],6)
                            oZ=round(trans_coords[5],6)
                            oW=round(trans_coords[6],6)
                            
                            # oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            # oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            # oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # oW=0.0 if math.isclose(round())
                            # self.get_logger().info(f'Part {i+1} Location: Color :{color}, Typee :{comp},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            colors = [0,1,2,3,4]
                            components = [10,11,12,13]
                            ort_var=(X,Y,Z,oX,oY,oZ,oW)
                            for colorr in colors:
                                for component in components:
                                    if color == colorr and comp == component:
                                        attr_name = f"_{colorr}_{component}"
                                        if hasattr(self, attr_name):
                                            target_list = getattr(self, attr_name)
                                            if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) \
                                                        and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[3],list(target_list)[i][3], abs_tol=0.01))\
                                                        and (math.isclose(ort_var[4],list(target_list)[i][4], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[5],list(target_list)[i][5], abs_tol=0.01)) \
                                                        and  (math.isclose(ort_var[6],list(target_list)[i][6], abs_tol=0.01)):
                                                        ispresent=True
                                                if ispresent==False:
                                                    target_list.add(ort_var)
                                                    self._dictionary.update({attr_name:target_list})
                                                # self.get_logger().debug(f"Updated {attr_name}: {ort_var}")
                    self.get_logger().debug(f"Added parts dict from right cam cb: {self._dictionary}") 
                    # self._left_bin_parts = len(msg._part_poses)
                    # self.get_logger().info(f"No. of parts in left bin: {self._left_bin_parts}") 



        except Exception as e:
            self.get_logger().error('Left ALC failed to process the part poses: %r' % (e,))
    
    def _kts1_camera_cb(self,msg):
        """
        Callback function for processing messages from the kts1 camera.

        Args:
            msg (AdvancedLogicalCameraImage): Message containing information about detected trays.

        Returns:
            None

        Notes:
            This method processes messages received from the kts1 camera.
            It extracts information about detected trays and calculates their positions and orientations.
            The transformed coordinates and orientations are then stored in the appropriate data structures.
        """
        try:
            if len(msg.tray_poses) == 0:
                self.get_logger().info('NO Tray DETECTED')
            else :
                # if self._kts1_trays != len(msg.tray_poses):
                    for i, tray_pose in enumerate(msg.tray_poses):
                        # self.get_logger().info(f'DETECTED part {i+1}: ')
                        I_D=tray_pose.id
                        self._part_parent_frame = "kts1_camera_frame"
                        self._part_frame = f"kts1_camera_tray_{i+1}_frame"
                        # self._right_broadcaster_part_pose(pose)
                        trans_coords =self.generate_transform(self._part_parent_frame, self._part_frame, tray_pose)
                        if trans_coords == "No":
                            self._tray_frame_flag = False
                        else:
                            self._tray_frame_flag = True
                            X=round(trans_coords[0],6)
                            Y=round(trans_coords[1],6)
                            Z=round(trans_coords[2],6)
                            oX=round(trans_coords[3],6)
                            oY=round(trans_coords[4],6)
                            oZ=round(trans_coords[5],6)
                            oW=round(trans_coords[6],6)
                            # oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # # oX=round(trans_coords[3],2)
                            # oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # # oY=round(trans_coords[4],2)
                            # oZ=oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Detected Tray{i+1} ID :{I_D},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            ort_var=(X,Y,Z,oX,oY,oZ,oW)
                            attr_name = f'_{I_D}'
                            if hasattr(self, attr_name):
                                target_list = getattr(self, attr_name)
                                if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) \
                                                        and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[3],list(target_list)[i][3], abs_tol=0.01))\
                                                        and (math.isclose(ort_var[4],list(target_list)[i][4], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[5],list(target_list)[i][5], abs_tol=0.01)) \
                                                        and  (math.isclose(ort_var[6],list(target_list)[i][6], abs_tol=0.01)):
                                                        ispresent=True
                                                if ispresent==False:
                                                    target_list.add(ort_var)
                                                    self._tray_dict.update({attr_name:target_list})
                                # if ort_var not in target_list:  
                                #     target_list.add(ort_var)
                                #     self._tray_dict.update({attr_name:target_list})
                                    # self.get_logger().debug(f"Updated {attr_name}: {ort_var}")
                    self.get_logger().debug(f"Added Trays dict from kts1 cb: {self._tray_dict}") 
                    # self._kts1_trays = len(msg.tray_poses)
                    # self.get_logger().info(f"No. of parts in kts1 {self._kts1_trays}") 


                        

                                
        except Exception as e:
            # print(traceback.format_exc())
            self.get_logger().error('kts1 ALC failed to process the tray poses: %r' % (e,))

                        

                                
        except Exception as e:
            # print(traceback.format_exc())
            self.get_logger().error('kts1 ALC failed to process the tray poses: %r' % (e,))
    
    def _kts2_camera_cb(self,msg):
        """
        Callback function for processing messages from the kts2 camera.

        Args:
            msg (AdvancedLogicalCameraImage): Message containing information about detected trays.

        Returns:
            None

        Notes:
            This method processes messages received from the kts2 camera.
            It extracts information about detected trays and calculates their positions and orientations.
            The transformed coordinates and orientations are then stored in the appropriate data structures.
        """
        try:
            if len(msg.tray_poses) == 0:
                self.get_logger().info('NO Tray DETECTED')
            else:
                # if self._kts2_trays != len(msg.tray_poses):
                    for i, tray_pose in enumerate(msg.tray_poses):
                        # self.get_logger().info(f'DETECTED part {i+1}: ')
                        I_D=tray_pose.id
                        self._part_parent_frame = "kts2_camera_frame"
                        self._part_frame = f"kts2_camera_tray_{i+1}_frame"
                        # self._right_broadcaster_part_pose(pose)
                        trans_coords =self.generate_transform(self._part_parent_frame, self._part_frame, tray_pose)
                        if trans_coords == "No":
                            self._tray_frame_flag = False
                        else:
                            self._tray_frame_flag = True
                            X=round(trans_coords[0],6)
                            Y=round(trans_coords[1],6)
                            Z=round(trans_coords[2],6)
                            oX=round(trans_coords[3],6)
                            oY=round(trans_coords[4],6)
                            oZ=round(trans_coords[5],6)
                            oW=round(trans_coords[6],6)
                            # oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # # oX=round(trans_coords[3],2)
                            # oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # # oY=round(trans_coords[4],2)
                            # oZ=oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Detected Tray{i+1} ID :{I_D},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            ort_var=(X,Y,Z,oX,oY,oZ,oW)

                            attr_name = f'_{I_D}'
                            if hasattr(self, attr_name):
                                target_list = getattr(self, attr_name)
                                if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) \
                                                        and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[3],list(target_list)[i][3], abs_tol=0.01))\
                                                        and (math.isclose(ort_var[4],list(target_list)[i][4], abs_tol=0.01)) \
                                                        and (math.isclose(ort_var[5],list(target_list)[i][5], abs_tol=0.01)) \
                                                        and  (math.isclose(ort_var[6],list(target_list)[i][6], abs_tol=0.01)):
                                                        ispresent=True
                                                if ispresent==False:
                                                    target_list.add(ort_var)
                                                    self._tray_dict.update({attr_name:target_list})
                                # if ort_var not in target_list:  
                                #     target_list.add(ort_var)
                                #     self._tray_dict.update({attr_name:target_list})
                                    # self.get_logger().debug(f"Updated {attr_name}: {ort_var}")
                    self.get_logger().debug(f"Added Trays dict from kts1 cb: {self._tray_dict}") 
                    # self._kts2_trays = len(msg.tray_poses)
                    # self.get_logger().info(f"No. of parts in kts2 {self._kts2_trays}") 


                    

                                

        except Exception as e:
            # print(traceback.format_exc())
            self.get_logger().error('kts2 ALC failed to process the Tray poses: %r' % (e,))

    
    
    def processing(self, order):
        """
        Process an order by assigning parts and trays to their respective positions.

        Args:
            order (Order): The order to be processed.

        Returns:
            int: Status code indicating the success of the processing operation. 
                - 0: No parts or trays found.
                - 1: Processing successful.

        Notes:
            This method processes an order by assigning parts and trays to their respective positions.
            It checks if the required parts and trays are present in the detected dictionaries.
            If found, it assigns the positions and orientations of the parts and trays to the order.
            The count of assigned parts and trays is updated for future references.
            If any part or tray is not found, it returns a status code indicating failure.
        """
        self.get_logger().info("Inside processing")
        
        if(len(self._dictionary)==0 or len(self._tray_dict)==0 ):
            self.get_logger().error("No Parts or Trays found.")
            return 0
            
        else:
            for trayid in order.tray:
                if(trayid in self._tray_dict):
                    self.get_logger().info(f'tray id : {trayid}')
                    if(trayid in self.trayCount):
                        self.get_logger().info('tray id in self.trayCount')
                        self.trayCount[trayid]+=1
                    else:
                        self.trayCount[trayid]=0
                        self.get_logger().info('first time tray')
                    order.tray[trayid] = list(self._tray_dict[trayid])[self.trayCount[trayid]]
                else:
                    return 0
            for part in order.parts:
                self.get_logger().info('Inside part for loop')
                if(part in self._dictionary):
                    self.get_logger().info(f'part : {part}')
                    if(part in self.partCount):
                        self.partCount[part]+=1
                        self.get_logger().info('part in self.partCount')
                    else:
                        self.partCount[part]=0
                        self.get_logger().info('first time part')
                    order.parts[part] = list(self._dictionary[part])[self.partCount[part]]
                else:
                    return 0
            return 1
    
    def timer_cb(self):
        """
        Timer callback function used to simulate the order fulfilling task and publish order lengths.

        Notes:
            This callback function is used to simulate the order fulfilling tasks.
            It also implements the functionality to handle high-priority orders.
            Additionally, it publishes the lengths of both low and high orders at a frequency of 1 Hz.

        # """
        # self.get_logger().info('🚀 Robot action timer callback called')
        # if self._kit_completed:
        #     self._main_timer.cancel()
        #     # check if the timer is cancelled
        #     if self._main_timer.is_canceled:
        #         self.get_logger().info("🏁 Timer cancelled")
        #     else:
        #         self.get_logger().info("🏁 Timer not cancelled")
                
        if self._all_frame_flag:
            low_orders_length = len(self._low_orders)
            high_orders_length = len(self._high_orders)
            self._send_order_length.data=[low_orders_length, high_orders_length]
            self._order_length_pub.publish(self._send_order_length)

            if(self._low_orders and len(self._high_orders)==0):
                self._counter+=1
                self.get_logger().info(f'Timer for lowerOrder: {self._counter}')
                self._lowCheck = self.processing(self._low_orders[0])
                self.get_logger().info(f'Low check value : {self._lowCheck}')
                if(self._lowCheck==1):
                    self._lowCheck=0
                    if self.tray_flag:
                        self.get_logger().info("In Tray Cycle............")
                    if not self._moving_robot_home:
                        self._move_robot_home()
                    if self._moved_robot_home:
                        if not self._moving_robot_to_table:
                            self._move_robot_to_table(MoveRobotToTable.Request.KTS1)
                        if not self._moving_robot_to_tray:
                            tray_id = MoveRobotToTray.Request.TRAY_ID1
                            tray_pose = Pose()
                            popped=self._low_orders[0]
                            for tray in popped.tray:
                                x,y,z,r,p,w=popped.tray[tray]
                            tray_pose.position.x = x
                            tray_pose.position.y = y
                            tray_pose.position.z = z
                            tray_pose.orientation.x = r
                            tray_pose.orientation.y = p
                            tray_pose.orientation.z = y
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
                                # if not self._moving_robot_to_table:
                                #         self._move_robot_to_table(MoveRobotToTable.Request.KTS2)
                            # if self._moved_robot_to_table:
                            #     if not self._vacuum_gripper_state.type == "part_gripper":
                            #         self.get_logger().info("Gripper change needed")
                            #         self._grip_change = True
                            #         self.gripper_change(self,"kts1","trays",ChangeGripper.Request.TRAY_GRIPPER)
                            #     else:
                            #         self._grip_change = False
                            # if (not self._entering_tool_changer) and self._grip_change:
                            #     self._enter_tool_changer("kts2", "parts")
                    
                            # if self._entered_tool_changer and self._grip_change:
                            #     if not self._changing_gripper:
                            #         self._change_gripper(ChangeGripper.Request.PART_GRIPPER)
                            
                            # if self._changed_gripper and self._grip_change:       
                            #     if not self._exiting_tool_changer:
                            #         self.get_logger().info("Exiting tool changer")
                            #         self._exit_tool_changer("kts2", "parts")
                            # if not self._grip_change:
                            #     if not self._activating_gripper:
                            #         self._activate_gripper()
                            # if self._activated_gripper:
                            #         self._entering_tool_changer = True
                            #         self._entered_tool_changer = False
                            #         self._changed_gripper = False
                            
                            # Move the robot to the tray
                            # TODO: Check whether the tray is picked up by using
                            # a subscriber to /ariac/floor_robot_gripper_state
                                    if not self._moving_robot_to_tray:  
                                        part.color = PickPartFromBin.Request.ORANGE
                                        part.type = PickPartFromBin.Request.BATTERY
                                        part_pose = Pose()
                                        for part in popped.pose:
                                            x,y,z,r,p,w=popped.pose[part]
                                        part_pose.position.x = x
                                        part_pose.position.y = y
                                        part_pose.position.z = z
                                        part_pose.orientation.x = r
                                        part_pose.orientation.y = p
                                        part_pose.orientation.z = w
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

            elif(self._high_orders or self._h_prior==True):
                self._h_counter+=1
                self.get_logger().info(f'Timer for higerOrder: {self._h_counter}')
                self._highCheck = self.processing(self._high_orders[0])
                self.get_logger().info(f'high check value : {self._highCheck}')
                if(self._highCheck == 1):
                    self._highCheck = 0
                    if self.tray_flag:
                        self.get_logger().info("In Tray Cycle............")
                    if not self._moving_robot_home:
                        self._move_robot_home()
                    if self._moved_robot_home:
                        if not self._moving_robot_to_table:
                            self._move_robot_to_table(MoveRobotToTable.Request.KTS1)
                            if not self._moving_robot_to_tray:
                                tray_id = MoveRobotToTray.Request.TRAY_ID1
                                tray_pose = Pose()
                                popped_h=self._high_orders[0]
                                for tray in popped_h.tray:
                                    x,y,z,r,p,w=popped_h.tray[tray]
                                tray_pose.position.x = x
                                tray_pose.position.y = y
                                tray_pose.position.z = z
                                tray_pose.orientation.x = r
                                tray_pose.orientation.y = p
                                tray_pose.orientation.z = y
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
                                # if not self._moving_robot_to_table:
                                #         self._move_robot_to_table(MoveRobotToTable.Request.KTS2)
                            # if self._moved_robot_to_table:
                            #     if not self._vacuum_gripper_state.type == "part_gripper":
                            #         self.get_logger().info("Gripper change needed")
                            #         self._grip_change = True
                            #         self.gripper_change(self,"kts1","trays",ChangeGripper.Request.TRAY_GRIPPER)
                            #     else:
                            #         self._grip_change = False
                            # if (not self._entering_tool_changer) and self._grip_change:
                            #     self._enter_tool_changer("kts2", "parts")
                    
                            # if self._entered_tool_changer and self._grip_change:
                            #     if not self._changing_gripper:
                            #         self._change_gripper(ChangeGripper.Request.PART_GRIPPER)
                            
                            # if self._changed_gripper and self._grip_change:       
                            #     if not self._exiting_tool_changer:
                            #         self.get_logger().info("Exiting tool changer")
                            #         self._exit_tool_changer("kts2", "parts")
                            # if not self._grip_change:
                            #     if not self._activating_gripper:
                            #         self._activate_gripper()
                            # if self._activated_gripper:
                            #         self._entering_tool_changer = True
                            #         self._entered_tool_changer = False
                            #         self._changed_gripper = False
                            
                            # Move the robot to the tray
                            # TODO: Check whether the tray is picked up by using
                            # a subscriber to /ariac/floor_robot_gripper_state
                                    if not self._moving_robot_to_tray:  
                                        part.color = PickPartFromBin.Request.ORANGE
                                        part.type = PickPartFromBin.Request.BATTERY
                                        part_pose = Pose()
                                        for part in popped_h.pose:
                                            x,y,z,r,p,w=popped_h.pose[part]
                                        part_pose.position.x = x
                                        part_pose.position.y = y
                                        part_pose.position.z = z
                                        part_pose.orientation.x = r
                                        part_pose.orientation.y = p
                                        part_pose.orientation.z = w
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
            else:
                pass
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
            
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        
        self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        if(msg.priority==False):
            self._low_orders.append(Orders(msg))
            self.get_logger().info(f'Received Low Priority Order at : {self._clock.now().nanoseconds/1e9}')
        else:
            self._high_orders.append(Orders(msg))
            self.get_logger().info(f'Received High Priority Order at :{self._clock.now().nanoseconds/1e9}')
            self._h_prior=True