import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from orders_sub_interface import Order
from ariac_msgs.srv import SubmitOrder

from ariac_msgs.msg import (
    AGVStatus,
)
from ariac_msgs.msg import (
    Order,
)



class Color:
    """
    A color class to modify the color of the text in logger.
    """
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    RESET = "\033[0m"


class OrderSubmisssionInterface(Node):
    """A ROS2 OrderSubmisssionInterface node that listens to AGVStatus messages on a specified topic and calls a service.

    This class creates a node that subscribes to the "/ariac/agv{n}_status" topics, expecting messages of type `ariac_msgs/msg/AGVStatus`.
    It logs the received messages to the ROS2 logger. This class also has a service client "/ariac/submit_order" service to submit order.
    
    Attributes:
        _submit_order_client (rclpy.client): The client object for calling the service.
        _agv_status_sub (rclpy.subscription.Subscription): The subscription object for receiving AGV status messages.
        _agv_status (ariac_msgs/msg/AGVStatus): A Int to store state values from /ariac/agv{n}_status topic.
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """    
    
 
    def __init__(self):
        super().__init__('OrderSubmisssionInterface')
        
        self.get_logger().info('Order Submisssion node started!')
        
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10)
        self. _submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order')
        
        self._agv_1_status_sub = self.create_subscription(AGVStatus, '/ariac/agv1_status', self.agv_1_status_cb, 10)
        self._agv_2_status_sub = self.create_subscription(AGVStatus, '/ariac/agv2_status', self.agv_2_status_cb, 10)
        self._agv_3_status_sub = self.create_subscription(AGVStatus, '/ariac/agv3_status', self.agv_3_status_cb, 10)
        self._agv_4_status_sub = self.create_subscription(AGVStatus, '/ariac/agv4_status', self.agv_4_status_cb, 10)
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10)
        self._id_agv_dict = {}
        self._id_submit = None
        self._submit_request = SubmitOrder.Request()
        self._agv_location_status = AGVStatus()
    
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic to link AGV number and order id.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores the order id and agv number.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        Order.id = msg.id
        Order.agv_number = msg.kitting_task.agv_number
        self._id_agv_dict[Order.agv_number] = Order.id
        
       
        
        
    def agv_1_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        self._agv_1_location_status = msg.location
        if self._agv_1_location_status == AGVStatus.WAREHOUSE:
            self._id_submit = self._id_agv_dict[1]
            self._submit_order_request(str(self._id_submit))
            self.get_logger().info('The AGV-1 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
            
    def agv_2_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        self._agv_2_location_status = msg.location
        if self._agv_2_location_status == AGVStatus.WAREHOUSE:
            self._id_submit = self._id_agv_dict[2]
            self._submit_order_request(str(self._id_submit))
            self.get_logger().info('The AGV-2 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def agv_3_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        self._agv_3_location_status = msg.location
        if self._agv_3_location_status == AGVStatus.WAREHOUSE:
            self._id_submit = self._id_agv_dict[3]
            self._submit_order_request(str(self._id_submit))
            self.get_logger().info('The AGV-3 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def agv_4_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        self._agv_4_location_status = msg.location
        if self._agv_4_location_status == AGVStatus.WAREHOUSE:
            self._id_submit = self._id_agv_dict[4]
            self._submit_order_request(str(self._id_submit))
            self.get_logger().info('The AGV has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def _submit_order_request(self,order_id:str) -> None:
        """Client request to send request to /ariac/submit_order service.
        
        This function is used to send the order id to submit order  to /ariac/submit_order service.
        
        """
        # Wait for the service to be available
        while not self._submit_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._submit_request.order_id = str(order_id)

        # Call the service asynchronously
        future = self._submit_order_client.call_async(self._submit_request)
        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)
        # Check if the response from the service is success or not.
        if future.result().success:
            self.get_logger().info('Order Submitted successfully')
        else:
            self.get_logger().warn('Order Submission failed')

    