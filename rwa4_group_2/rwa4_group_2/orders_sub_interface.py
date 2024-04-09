import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32, Int32MultiArray
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
        _AGV_ID_pub (rclpy.publisher.Publisher): The publisher object for publishing fullfilled AGV Ids on topic /fullfilled_agv_id
        _order_length_pub (rclpy.publisher.Publisher):  The publisher object for publishing low priority and high prioirity order lengths /order_length
        _timer (rclpy.time.Timer): creating timer for timer_cb callback function at the 1hz frequency. This timer is used for faking the order fulfillment
        _clock (clock): used for logging the sim time while receveing the orders
        _counter (int32): A counter for keeping count of 15 sec while fullfilling the low priority order
        _h_counter (int32): A counter for keeping count of 15 sec while fullfilling the high priority order
        _h_prior (Bool):  A Flag of high priority task
        _send_msgs (Int32): An attritube to store the AGV ID and later used to pulish it to the topic /fullfilled_agv_id
        _send_order_length (Int32 Multi Array): An attribute to store the lengths of the low and high priority order lists
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """
    def __init__(self):
        super().__init__('OrderSubInterface')
        #Subscribers
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10)
        #Publishers
        self._AGV_ID_pub = self.create_publisher(Int32, "fullfilled_agv_id", 10)    
        self._order_length_pub = self.create_publisher(Int32MultiArray, "/order_length", 1)
        #timers
        self._timer=self.create_timer(1,self.timer_cb)    
        #Attributes
        self._clock = self.get_clock()
        self._counter=0
        self._h_counter=0
        self._h_prior=False
        self._low_orders = []   
        self._high_orders = []
        self._send_msg= Int32()
        self._send_order_length = Int32MultiArray()

   
    def timer_cb(self):
        """ A timer callback used to fake the order fullfilling task of 15 sec
        Also implemented the high priority task to take over the ongoing low priority task
        Also publishes the length of the orders(low as well as high) at 1hz frequency
        """

        low_orders_length = len(self._low_orders)
        high_orders_length = len(self._high_orders)
        self._send_order_length.data=[low_orders_length, high_orders_length]
        self._order_length_pub.publish(self._send_order_length)

        if(self._low_orders and len(self._high_orders)==0 ):
            self._counter+=1
            self.get_logger().info(f'Timer for lowerOrder: {self._counter}')

            if (self._counter%15==0):
                poped=self._low_orders.pop(0)
                self.get_logger().info(f'fulfilling lower order {poped.id} on AGV number {poped.agv_number}')
                self._send_msg.data= poped.agv_number
                self._AGV_ID_pub.publish(self._send_msg)
                self.get_logger().info('PUBLISHED Lower  Order')



        elif(self._high_orders or self._h_prior==True):
            self._h_counter+=1
            self.get_logger().info(f'Timer for higerOrder: {self._h_counter}')

            if (self._h_counter%15==0):
                poped=self._high_orders.pop(0)
                self.get_logger().info(f'fulfilling higher order {poped.id} on AGV number {poped.agv_number}')
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
        
        self.get_logger().info(f'Received an order with order id: {msg.id} and priority: {msg.priority}')
        
        #Appending Low prioirity orders in the _low_orders list
        if(msg.priority==False):
            self._low_orders.append(Orders(msg))
            self.get_logger().info(f'Received Low Priority Order at : {self._clock.now().nanoseconds/1e9}')
        
        #Appending Low prioirity orders in the _high_orders list
        else:
            self._high_orders.append(Orders(msg))
            self.get_logger().info(f'Received High Priority Order at :{self._clock.now().nanoseconds/1e9}')
            self._h_prior=True

            
