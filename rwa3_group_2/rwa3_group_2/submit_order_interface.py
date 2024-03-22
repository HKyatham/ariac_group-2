import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from ariac_msgs.msg import (
    AGVStatus,
)

from ariac_msgs.srv import SubmitOrder

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
        
        self. _submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order')
        
        self._agv_1_status_sub = self.create_subscription(AGVStatus, '/ariac/agv1_status', self.agv_status_cb, 10)
        self._agv_2_status_sub = self.create_subscription(AGVStatus, '/ariac/agv2_status', self.agv_status_cb, 10)
        self._agv_3_status_sub = self.create_subscription(AGVStatus, '/ariac/agv3_status', self.agv_status_cb, 10)
        self._agv_4_status_sub = self.create_subscription(AGVStatus, '/ariac/agv4_status', self.agv_status_cb, 10)
        
        self._agv_location_status = AGVStatus()
        
        
    def agv_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        self._agv_location_status = msg.location
        if self._agv_location_status == AGVStatus.WAREHOUSE:
            self.get_logger().info('The AGV has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def submit_order(self):
        """Client request to send request to /ariac/submit_order service.
        
        This function is used to send an empty SubmitOrder request to /ariac/submit_order service.
        
        """
        self.get_logger().info('Submit order client called')
        
        while self._agv_location_status != AGVStatus.WAREHOUSE:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                self.get_logger().error('KeyboardInterrupt received!')
        # Check if the AGV reached WAREHOUSE, if ready send the request to service.
        if self._agv_location_status == AGVStatus.WAREHOUSE:
            # Creating Request
            request = SubmitOrder.Request()
            # Async call to the service.
            future = self._submit_order_client.call_async(request)
            
            self.get_logger().info('Inside WAREHOUSE if case.')
            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)
            # Check if the response from the service is success or not.
            if future.result().success:
                self.get_logger().info('Order Submitted successfully')
            else:
                 self.get_logger().warn('Order Submission failed')

    