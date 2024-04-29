

Skip to content
Using University of Maryland, College Park Mail with screen readers

Conversations

Unread
 
1–1 of 1
 

Everything else
 
1–10 of 2,472
 
Using 10.44 GB
Program Policies
Powered by Google
Last account activity: 31 minutes ago
Details
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
from ariac_msgs.msg import (
    Order,
    AdvancedLogicalCameraImage
)
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


# class Part():
#     def __init__(self,part):
#         self.type = part.part.type
#         self.color = part.part.color
#         self.quadrant = part.quadrant

# class Orders():
#     def __init__(self, msg: Order):
#         self.id = msg.id
#         self.tray = {f'_{msg.kitting_task.tray_id}':set()}
#         self.priority = msg.priority
#         self.agv_number =  msg.kitting_task.agv_number
#         self.destination = msg.kitting_task.destination
#         self.parts = {}
#         for part in msg.kitting_task.parts:
#             p=Part(part)
#             self.parts[f'_{p.color}_{p.type}']= set()
                    
                    
                
            
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

        self._kts1_trays=0
        self._kts2_trays=0
        self._right_bin_parts=0
        self._left_bin_parts=0
        
    def quaternion_to_euler(self,quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).

        Args:
            quaternion: Quaternion object representing the orientation.

        Returns:
            Tuple containing the euler angles (roll, pitch, yaw) in radians.

        Notes:
            Roll is rotation around x in radians (counterclockwise).
            Pitch is rotation around y in radians (counterclockwise).
            Yaw is rotation around z in radians (counterclockwise).
        """
        
        x,y,z,w=quaternion.x, quaternion.y, quaternion.z, quaternion.w
        

        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        
        sinr_cosp = +2.0 * (w * x + y * z)
        cosr_cosp = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = +2.0 * (w * y - z * x)
        if (math.fabs(sinp) >= 1):
            pitch = math.copysign(math.M_PI / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = +2.0 * (w * z + x * y)
        cosy_cosp = +1.0 - 2.0 * (y * y + z * z)  
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

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
            # self.get_logger().info(f'DETECTED part  orientation: {quat}')

            r,p,y=self.quaternion_to_euler(quat)
            # self.get_logger().info(f"Transform between world and {child}: \n")
                                #    + str(t.transform.translation)+ str(r)+ str(p)+ str(y))
            transformed_part_pose=[transformed_x,transformed_y,transformed_z,r,p,y]
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
                            oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Part {i+1} Location: Color :{color}, Typee :{comp},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            colors = [0,1,2,3,4]
                            components = [10,11,12,13]
                            ort_var=(X,Y,Z,oX,oY,oZ)
                            for colorr in colors:
                                for component in components:
                                    if color == colorr and comp == component:
                                        attr_name = f"_{colorr}_{component}"
                                        if hasattr(self, attr_name):
                                            target_list = getattr(self, attr_name)
                                            if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)):
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
                            oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            oZ=oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Part {i+1} Location: Color :{color}, Typee :{comp},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            colors = [0,1,2,3,4]
                            components = [10,11,12,13]
                            ort_var=(X,Y,Z,oX,oY,oZ)
                            for colorr in colors:
                                for component in components:
                                    if color == colorr and comp == component:
                                        attr_name = f"_{colorr}_{component}"
                                        if hasattr(self, attr_name):
                                            target_list = getattr(self, attr_name)
                                            if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)):
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
                            oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            oZ=oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Detected Tray{i+1} ID :{I_D},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            ort_var=(X,Y,Z,oX,oY,oZ)
                            attr_name = f'_{I_D}'
                            if hasattr(self, attr_name):
                                target_list = getattr(self, attr_name)
                                if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)):
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
                            oX=0.0 if math.isclose(round(trans_coords[3],3),0, abs_tol=0.01) == True else round(trans_coords[3],3)
                            # oX=round(trans_coords[3],2)
                            oY=0.0 if math.isclose(round(trans_coords[4],3),0, abs_tol=0.01) == True else round(trans_coords[4],3)
                            # oY=round(trans_coords[4],2)
                            oZ=oZ=3.14 if round(trans_coords[5],2) == -3.14 else round(trans_coords[5],2)
                            # self.get_logger().info(f'Detected Tray{i+1} ID :{I_D},')
                            # self.get_logger().info(f'Part {i+1} Location: X :{X}, Y :{Y}, Z :{Z}, oX :{oX}, oY :{oY}, oZ :{oZ} ')
                            ort_var=(X,Y,Z,oX,oY,oZ)

                            attr_name = f'_{I_D}'
                            if hasattr(self, attr_name):
                                target_list = getattr(self, attr_name)
                                if ort_var not in target_list:
                                                ispresent=False
                                                
                                                for i in range(len(target_list)):
                                                    if(math.isclose(ort_var[0],list(target_list)[i][0], abs_tol=0.01)) and(math.isclose(ort_var[1],list(target_list)[i][1], abs_tol=0.01)) and (math.isclose(ort_var[2],list(target_list)[i][2], abs_tol=0.01)):
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
    
    # def processing(self, order):
    def processing(self,msg: Order):
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
            self.id = msg.id
            self.tray = {f'_{msg.kitting_task.tray_id}':set()}
            self.priority = msg.priority
            self.agv_number =  msg.kitting_task.agv_number
            self.destination = msg.kitting_task.destination
            self.parts = {}
            # for part in msg.kitting_task.parts:
            #     p=Part(part)
            #     self.parts[f'_{p.color}_{p.type}']= set()

            #Fetches the tray ID(trayid) number from params.tray
            for trayid in self.tray:
                #Checks if trayid is present in the key values of dictionary self._tray_dict
                if(trayid in self._tray_dict):
                    self.get_logger().info(f'tray id : {trayid}')
                    # self.get_logger().info(f"Indide Processing printting tray dict : {self._tray_dict}") 
                    self.get_logger().info(f"*****************************************************************")  

                    # self.get_logger().info(f"Inside Processing printing dict of parts: {self._dictionary}")  


                    ### BASIC Function of the below condi. block is to to get the index value ###
                    #Checks if trayid is present in the key values of dictionary self._traycount
                    #initially it will go into else block where it will create key as trayid and its value as 0
                    #Then if the same trayId is repeated it will go in the above 'if block' and
                    # will increment the value by 1 for that given key(trayId)
                    if(trayid in self.trayCount):
                        self.get_logger().info('tray id in self.trayCount')
                        self.trayCount[trayid]+=1
                    else:
                        self.trayCount[trayid]=0
                        self.get_logger().info('first time tray')
                    #Once got the index values, update the dictionary with of  params.tray with the tray poses
                    # As self._tray_dict dictionay may have multiple values in the same key , we are iterating over the values 
                    #with the help of index caloculatd in above block
                    self.tray[trayid] = {list(self._tray_dict[trayid])[self.trayCount[trayid]]}
                    self.get_logger().info(f'While adding part to dic in processing {self.tray}')

                else:
                    return 0
            for part in msg.kitting_task.parts:
                self.type = part.part.type
                self.color = part.part.color
                self.quadrant = part.quadrant
                # self.parts[f'_{self.color}_{self.type}']= set()
                self.np=f'_{self.color}_{self.type}'
            # for part in self.parts:
                self.get_logger().info('Inside part for loop')
                if(self.np in self._dictionary):
                    self.get_logger().info(f'part : {self.np}')
                    if(self.np in self.partCount):
                        self.partCount[self.np]+=1
                        self.get_logger().info('part in self.partCount')
                        self.parts[self.np].add(list(self._dictionary[self.np])[self.partCount[self.np]])

                    else:
                        self.partCount[self.np]=0
                        self.get_logger().info('first time part')
                        self.parts[self.np]= {list(self._dictionary[self.np])[self.partCount[self.np]]}

                    # if self.np in self.parts:
                    #     self.parts[self.np].add(list(self._dictionary[self.np])[self.partCount[self.np]])
                    #     self.get_logger().info(f' ADDDD ing part {self.np} to dic in processing {self.parts[self.np]}')

                    # else:
                    #     self.parts[self.np]= list(self._dictionary[self.np])[self.partCount[self.np]]

                        
                    # order.parts[part].add(list(self._dictionary[part])[self.partCount[part]])
                    self.get_logger().info(f'While adding part to dic in processing {self.parts}')

                else:
                    print("0000000")
                    return 0
            return 1
    
    def timer_cb(self):
        """
        Timer callback function used to simulate the order fulfilling task and publish order lengths.

        Notes:
            This callback function is used to simulate the order fulfilling tasks.
            It also implements the functionality to handle high-priority orders.
            Additionally, it publishes the lengths of both low and high orders at a frequency of 1 Hz.

        """
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
                ################################################################################################################
                if(self._lowCheck==1):
                    self._lowCheck=0
                    poped=self._low_orders.pop(0)
                    self.get_logger().info(f'fulfilling lower order {poped.id} on AGV number {poped.kitting_task.agv_number}')
                    self._send_msg.data= poped.kitting_task.agv_number
                    self._AGV_ID_pub.publish(self._send_msg)
                    self.get_logger().info('PUBLISHED Lower  Order')
                    self.get_logger().info(f'- {poped.id}')
                    # for tray in poped.tray:
                    self.nt=f'_{poped.kitting_task.tray_id}'

                    self.get_logger().info(f'Printinh tray ID : {self.tray}')
                    
                    # x,y,z,r,p,w=self.tray[self.nt]
                    # xyz=self.tray[f'_{poped.kitting_task.tray_id}']

                    # id = str(poped.kitting_task.tray_id)
                    # id = id.replace('_',"")
                    self.get_logger().info(f'  - ID: {self.tray}')
                    self.get_logger().info(f'  - Position (xyz): [{x}, {y}, {z}]')
                    self.get_logger().info(f'  - Orientation (rpy): [{r}, {p}, {w}]')
                    for part in poped.kitting_task.parts:
                        self.type = part.part.type
                        self.color = part.part.color
                        self.quadrant = part.quadrant
                        self.npp=f'_{self.color}_{self.type}'
                        for val in self.parts[self.npp]:
                            x,y,z,r,p,w=val

                            # x,y,z,r,p,w=poped.parts[part]
                            color, type = str(part).split("_")[1::]
                            color = ColorTypeMapper._part_colors[int(color)]
                            type = ColorTypeMapper._part_types[int(type)]
                            # print(ColorTypeMapper._part_colors.keys())
                            self.get_logger().info(f'  - {color} {type}:')
                            self.get_logger().info(f'    - Position (xyz): [{x}, {y}, {z}]')
                            self.get_logger().info(f'    - Orientation (rpy): [{r}, {p}, {w}]')

                #################################################################################################################
                # if(self._lowCheck==1):
                #     self._lowCheck=0
                #     poped=self._low_orders.pop(0)
                #     self.get_logger().info(f'fulfilling lower order {poped.id} on AGV number {poped.agv_number}')
                #     self._send_msg.data= poped.agv_number
                #     self._AGV_ID_pub.publish(self._send_msg)
                #     self.get_logger().info('PUBLISHED Lower  Order')
                #     self.get_logger().info(f'- {poped.id}')
                #     for tray in poped.tray:
                #         x,y,z,r,p,w=poped.tray[tray]
                #         id = str(tray)
                #         id = id.replace('_',"")
                #         self.get_logger().info(f'  - ID: {id}')
                #         self.get_logger().info(f'  - Position (xyz): [{x}, {y}, {z}]')
                #         self.get_logger().info(f'  - Orientation (rpy): [{r}, {p}, {w}]')
                #     for part in poped.parts:
                #         for val in poped.parts[part]:
                #             x,y,z,r,p,w=val

                #             # x,y,z,r,p,w=poped.parts[part]
                #             color, type = str(part).split("_")[1::]
                #             color = ColorTypeMapper._part_colors[int(color)]
                #             type = ColorTypeMapper._part_types[int(type)]
                #             # print(ColorTypeMapper._part_colors.keys())
                #             self.get_logger().info(f'  - {color} {type}:')
                #             self.get_logger().info(f'    - Position (xyz): [{x}, {y}, {z}]')
                #             self.get_logger().info(f'    - Orientation (rpy): [{r}, {p}, {w}]')

            # elif(self._high_orders or self._h_prior==True):
            #     self._h_counter+=1
            #     self.get_logger().info(f'Timer for higerOrder: {self._h_counter}')
            #     self._highCheck = self.processing(self._high_orders[0])
            #     self.get_logger().info(f'high check value : {self._highCheck}')
            #     if(self._highCheck == 1):
            #         self._highCheck = 0
            #         poped=self._high_orders.pop(0)
            #         self.get_logger().info(f'fulfilling higher order {poped.id} on AGV number {poped.agv_number}')
            #         self._send_msg.data= poped.agv_number
            #         self._AGV_ID_pub.publish(self._send_msg)
            #         self._h_prior=False
            #         self.get_logger().info('PUBLISHED Higer  Order')
            #         self.get_logger().info(f'- {poped.id}')
            #         for tray in poped.tray:
            #             x,y,z,r,p,w=poped.tray[tray]
            #             id = str(tray)
            #             id = id.replace('_',"")
            #             self.get_logger().info(f'  - ID: {id}')
            #             self.get_logger().info(f'  - Position (xyz): [{x}, {y}, {z}]')
            #             self.get_logger().info(f'  - Orientation (rpy): [{r}, {p}, {w}]')
            #         for part in poped.parts:
            #             for val in poped.parts[part]:
            #                 x,y,z,r,p,w=val
            #                 # x,y,z,r,p,w=poped.parts[part]
            #                 color, type = str(part).split("_")[1::]
            #                 color = ColorTypeMapper._part_colors[int(color)]
            #                 type = ColorTypeMapper._part_types[int(type)]
            #                 # print(ColorTypeMapper._part_colors.keys())
            #                 self.get_logger().info(f'  - {color} {type}:')
            #                 self.get_logger().info(f'    - Position (xyz): [{x}, {y}, {z}]')
            #                 self.get_logger().info(f'    - Orientation (rpy): [{r}, {p}, {w}]')
                    
            else:
                pass

            
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        
        self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        
        
        #Appending Low prioirity orders in the _low_orders list
        if(msg.priority==False):
            self.get_logger().info(f'LOW PRIO ORDERS DETAILS {len(msg.kitting_task.parts)}')
            # self._low_orders.append(Orders(msg))
            self._low_orders.append(msg)

            self.get_logger().info(f'Received Low Priority Order at : {self._clock.now().nanoseconds/1e9}')
        
        #Appending Low prioirity orders in the _high_orders list
        else:
            self.get_logger().info(f'HIGH PRIO ORDERS DETAILS {len(msg.kitting.parts)}')
            # self._high_orders.append(Orders(msg))
            self._high_orders.append(msg)
            self.get_logger().info(f'Received High Priority Order at :{self._clock.now().nanoseconds/1e9}')
            self._h_prior=True
