import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from ariac_msgs.msg import (
    Order,
)
class Part():
    def __init__(self,part):
        self.type = part.part.type
        self.color = part.part.color
        self.quadrant = part.quadrant

class Orders():
    def __init__(self, msg: Order):
        self.id = msg.id
        self.priority = msg.priority
        self.agv_number =  msg.kitting_task.agv_number
        self.destination = msg.kitting_task.destination
        self.parts = []
        for part in msg.kitting_task.parts:
            self.parts.append(Part(part))
            
class OrderSubInterface(Node):
    """A ROS2 OrderSubInterface node that listens to Order messages on a specified topic.

    This class creates a node that subscribes to the "/ariac/orders" topics, expecting messages of type `ariac_msgs/msg/Order`.
    It logs the part of received messages to the ROS2 logger and stores the message.

    Attributes:
        _orders_sub (rclpy.subscription.Subscription): The subscription object for receiving Order messages.
        _low_orders (List): A list object to store low priority orders.
        _high_orders (List): A list object to store high priority orders.
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """
    def __init__(self):
        super().__init__('OrderSubInterface')
        
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10)
        
        self._low_orders = []
        
        self._high_orders = []
        
    
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        # self.get_logger().info(f'Received an order with agv number : {msg.kitting_task.agv_number} and destination: {msg.kitting_task.destination}')
            # self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        if(msg.priority):
            self._high_orders.append(Orders(msg))
        else:
            self._low_orders.append(Orders(msg))