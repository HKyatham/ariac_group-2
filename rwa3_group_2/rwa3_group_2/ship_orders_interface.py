
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ariac_msgs.msg import Order
from std_msgs.msg import Int32
# from ship_order_srv.srv import MoveAGV  
from ariac_msgs.srv import (
    MoveAGV)

class AGVControlClient(Node):
    """
    AGVControlClient is a node that listens to the topic /fullfilled_agv_id and processes them by locking the tray and moving the AGV to destination.
    Attributes:
        subscription (rclpy.subscription.Subscription): The subscription object for receiving fullfilled agv_id messages.
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """
    def __init__(self):
        super().__init__('agv_control_client')
        self.subscription = self.create_subscription(Int32, 'fullfilled_agv_id', self.order_callback, 10)

        self._agv_orders = [] 

    @property

    def orders(self):
        """
        A getter function to get the list of AGV IDs
        """
        return self._agv_orders

    def order_callback(self, msg):
        """
        Handle incoming messages on the "/fullfilled_agv_id" topic.

        This function is called when a new message is received on the "/fullfilled_agv_id" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the fullfilled agv ids data.

        """
        agv_id = msg.data
        self.get_logger().info(f'Received order for AGV {agv_id}')
        self._agv_orders.append(agv_id)


   
    def lock_tray(self, num):

        '''
        Lock the tray of an AGV and parts on the tray. This will prevent tray and parts from moving during transport.
        Args:
            num (int):  AGV number
        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        # Create a client to send a request to the `/ariac/agv{num}_lock_tray` service

        tray_locker = self.create_client(
            Trigger,
            f'/ariac/agv{num}_lock_tray'
        )
        self.get_logger().info('Inside lock tray after creating clinet')

        # Build the request
        request = Trigger.Request()
        # Send the request
        future = tray_locker.call_async(request)
        self.get_logger().info('Inside lock tray after Triggering request and waiting for response')

        # Wait for the response
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error
        

        # Check the response
        if future.result().success:
            self.get_logger().info(f'Locked AGV{num}\'s tray')
            return future.result()
        else:
            self.get_logger().warn('Unable to lock tray')
            return future.result()


    
    
    def move_agv(self, num, station):

        '''
        Move an AGV to an assembly station.
        Args:
            num (int): AGV number
            station (int): Assembly station number
        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        # Create a client to send a request to the `/ariac/move_agv` service.
        mover = self.create_client(
            MoveAGV,
            f'/ariac/move_agv{num}')
        
        self.get_logger().info('Inside MOVE AGV after creating clinet')

        # Create a request object.
        request = MoveAGV.Request()
        # Set the request location.
        request.location = MoveAGV.Request.WAREHOUSE
        # Send the request.
        future = mover.call_async(request)

        self.get_logger().info('Inside MOVE AGV after Triggering request and waiting for response')

        # Wait for the server to respond.
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error
        # Check the result of the service call.
        if future.result().success:
            self.get_logger().info(f'Moved AGV{num} to {station}')
            return future.result()

        else:
            self.get_logger().warn(future.result().message)
            return future.result()


    
