import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
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
        # self._clock_sub = self.create_subscription(Clock,'/clock',self.clock,10)

        # self._low_priority_pub = self.create_publisher(String, "low_priority", 10)
        self._AGV_ID_pub = self.create_publisher(Int32, "fullfilled_agv_id", 10)    

        self._timer=self.create_timer(1,self.timer_cb)    
        
        self._clock = self.get_clock()
        self._counter=0
        self._h_counter=0
        self._h_prior=False
        self._low_orders = []   
        self._high_orders = []
        self._send_msg= Int32()

    # def clock(self, msg):
    #     self.time=msg.sec
    def timer_cb(self):
        if(self._low_orders and len(self._high_orders)==0 ):
            self._counter+=1
            self.get_logger().info(f'Inside lowerOrder: {self._counter}')

            if (self._counter%5==0):
                # self.get_logger().info('fullfiimg lower order')
                poped=self._low_orders.pop(0)
                self.get_logger().info(f'fullfiimg lower order {poped.id} on AGV number {poped.agv_number}')
                self._send_msg.data= poped.agv_number
                self._AGV_ID_pub.publish(self._send_msg)
                self.get_logger().info('PUBLISHED Lower  Order')



        elif(self._high_orders or self._h_prior==True):
            self._h_counter+=1
            self.get_logger().info(f'Inside higerOrder: {self._h_counter}')

            if (self._h_counter%5==0):
                # self.get_logger().info('fullfiimg higher order:')
                poped=self._high_orders.pop(0)
                self.get_logger().info(f'fullfiimg higher order {poped.id} on AGV number {poped.agv_number}')
                self._send_msg.data= poped.agv_number
                self._AGV_ID_pub.publish(self._send_msg)
                self._h_prior=False
                self.get_logger().info('PUBLISHED Higer  Order')
                
        else:
            pass


        
    
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        self.get_logger().info('Received some  Order')
        
        self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        # self.get_logger().info(f'Received an order with agv number : {msg.kitting_task.agv_number} and destination: {msg.kitting_task.destination}')
            # self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
                
        if(msg.priority==False):
            self._low_orders.append(Orders(msg))
            self.get_logger().info(f'Received Low Priority Order at : {self._clock.now().nanoseconds/1e9}')
        

        else:
            self._high_orders.append(Orders(msg))
            self.get_logger().info(f'Received High Priority Order at :{self._clock.now().nanoseconds/1e9}')
            self._h_prior=True

            
