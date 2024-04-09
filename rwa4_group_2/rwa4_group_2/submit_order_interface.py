import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.srv import SubmitOrder
from ariac_msgs.msg import AGVStatus, Order
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class Color:
    """
    A color class to modify the color of the text in logger.
    """
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    RESET = "\033[0m"


class OrderSubmissionInterface(Node):
    """A ROS2 OrderSubmissionInterface node that listens to AGVStatus messages on a specified topic and calls a service.

    This class creates a node that subscribes to the "/ariac/agv{n}_status" topics, expecting messages of type `ariac_msgs/msg/AGVStatus`.
    It also subscribes to the "/ariac/orders" topics, expecting messages of type `ariac_msgs/msg/Order`.
    It also has a publisher that publishes to "/complete_order" topic of type Bool.
    It logs the received messages to the ROS2 logger. This class also has a service client "/ariac/submit_order" service to submit order.
    
    Attributes:
        _submit_order_client (rclpy.client): The client object for calling the service.
        _agv_{n}_status_sub (rclpy.subscription.Subscription): The subscription object for receiving AGV status messages.
        _orders_sub (rclpy.subscription.Subscription): The subscription object for receiving Order messages.
        _agv_status (ariac_msgs/msg/AGVStatus): A Int to store state values from /ariac/agv{n}_status topic.
        _submit_request (ariac_msgs.srv.SubmitOrder.Request): An instance of `ariac_msgs.srv.SubmitOrder.Request` to store request object for submitting orders.
        _complete_order_pub (rclpy.publisher.Publisher): The publisher object for publishing to "/complete_order" topic of type `std_msgs.msg.Bool`.
        _complete_order_timer (rclpy.timer.Timer): The timer object for calling the `complete_order_publish_message()` method.
        _agv_location_status (ariac_msgs.msg.AGVStatus): An instance of `ariac_msgs.msg.AGVStatus` to store AGV status messages.
        _submitted_orders (set): A set to store submitted orders.
        _id_agv_dict (dict): A dictionary to store AGV numbers and corresponding order IDs.
        _id_{n}_submit (str): Variables to store order IDs for each AGV.
        _order_id (str): A variable to store the order ID.
        _agv_number (int): A variable to store the AGV number.
        _order_id_list (set): A set to store order IDs.
        _total_orders (int): A counter to track the total number of orders.
        _complete_order (bool): A boolean to track order completion.   
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """    
    mutex_group1 = MutuallyExclusiveCallbackGroup()
    reentrant_group1 = ReentrantCallbackGroup()

    
 
    def __init__(self):
        super().__init__('OrderSubmissionInterface')
        
        self.get_logger().info('Order Submission node started!')
        
        # Creating subscriber to '/ariac/orders' topic
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10,callback_group= OrderSubmissionInterface.mutex_group1)
        
        # Creating subcribers to '/ariac/agv{n}_status' topic
        self._agv_1_status_sub = self.create_subscription(AGVStatus, '/ariac/agv1_status', self.agv_1_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_2_status_sub = self.create_subscription(AGVStatus, '/ariac/agv2_status', self.agv_2_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_3_status_sub = self.create_subscription(AGVStatus, '/ariac/agv3_status', self.agv_3_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_4_status_sub = self.create_subscription(AGVStatus, '/ariac/agv4_status', self.agv_4_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        
        
        # Creating publisher to "/complete_order" topic
        self._complete_order_pub = self.create_publisher(Bool, "/complete_order", 1)
        
        # Creating service client to submit order
        self. _submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order')
       
        # timer to call complete_order_publish_message()
        self._complete_order_timer = self.create_timer(1, self.complete_order_publish_message)

        # Creating request object
        self._submit_request = SubmitOrder.Request()
        self._agv_location_status = AGVStatus()
        
        # set to store submitted orders
        self._submitted_orders = set() 
        # Dictonary to store agv number and order id
        self._id_agv_dict = {}
        
        # varible (str) to store order-id in each AGV
        self._id_1_submit = None
        self._id_2_submit = None
        self._id_3_submit  = None
        self._id_4_submit = None
        # varible (str) to store order id
        self._order_id = None
        # variable (int) to tore agv number
        self._agv_number = None
        # set of order ids
        self._order_id_list = set()
        # counter to track total orders
        self._total_orders = 0
        # Boolean to track order completion.
        self._complete_order= bool()
    
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic to link AGV number and order id.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores the order id and agv number.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        # storing order id and agv number in a dictionary and logging the same
        self._order_id= msg.id
        self._agv_number = msg.kitting_task.agv_number
        self._id_agv_dict[self._agv_number] = self._order_id
        self.get_logger().info(f'Order ID is : {self._order_id}')
        # Augmenting the total orders counter
        self._total_orders += 1
        self.get_logger().info(f'Total Order Count : {self._total_orders}')
        self._complete_order= False
       
        
        
    def agv_1_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # storing agv location
        self._agv_1_location_status = msg.location
        # checking if agv is in warehouse and if order id is in submitted set.
        if self._agv_1_location_status == AGVStatus.WAREHOUSE:
            self._id_1_submit = self._id_agv_dict.get(1)
            if self._id_1_submit not in self._submitted_orders:
                # calling service client to submit order and storing order id for future reference
                self._submitted_orders.add(self._id_1_submit)
                self._order_id_list.add(self._id_1_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_1_submit}')
                self._submit_order_request(self._id_1_submit)
                self.get_logger().info(Color.GREEN + 'The AGV-1 has reached WAREHOUSE' + Color.RESET)
            
    def agv_2_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # storing agv location
        self._agv_2_location_status = msg.location
        # checking if agv is in warehouse and if order id is in submitted set.
        if self._agv_2_location_status == AGVStatus.WAREHOUSE:
            self._id_2_submit = self._id_agv_dict.get(2)
            if self._id_2_submit not in self._submitted_orders:
                # calling service client to submit order and storing order id for future reference
                self._submitted_orders.add(self._id_2_submit)
                self._order_id_list.add(self._id_2_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_2_submit}')
                self._submit_order_request(self._id_2_submit)
                self.get_logger().info(Color.GREEN + 'The AGV-2 has reached WAREHOUSE' + Color.RESET)

            
    def agv_3_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # storing agv location
        self._agv_3_location_status = msg.location
        # checking if agv is in warehouse and if order id is in submitted set.
        if self._agv_3_location_status == AGVStatus.WAREHOUSE:
            self._id_3_submit = self._id_agv_dict.get(3)
            if self._id_3_submit not in self._submitted_orders:
                # calling service client to submit order and storing order id for future reference
                self._submitted_orders.add(self._id_3_submit)
                self._order_id_list.add(self._id_3_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_3_submit}')
                self._submit_order_request(self._id_3_submit)
                self.get_logger().info(Color.GREEN + 'The AGV-3 has reached WAREHOUSE' + Color.RESET)

            
    def agv_4_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # storing agv location
        self._agv_4_location_status = msg.location
        # checking if agv is in warehouse and if order id is in submitted set.
        if self._agv_4_location_status == AGVStatus.WAREHOUSE:
            self._id_4_submit = self._id_agv_dict.get(4)
            if self._id_4_submit not in self._submitted_orders:
                # calling service client to submit order and storing order id for future reference
                self._submitted_orders.add(self._id_4_submit)
                self._order_id_list.add(self._id_4_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_4_submit}')
                self._submit_order_request(self._id_4_submit)
                self.get_logger().info(Color.GREEN + 'The AGV-4 has reached WAREHOUSE' + Color.RESET)
                
            
    def _submit_order_request(self,order_id:str) -> None:
        """Client request to send request to /ariac/submit_order service.
        
        This function is used to send the order id to submit order  to /ariac/submit_order service.
        
        """
        # Wait for the service to be available
        while not self._submit_order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Fill in the request object
        self._submit_request.order_id = order_id

        # Call the service asynchronously
        future = self._submit_order_client.call_async(self._submit_request)
        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.submit_future_callback)

    def submit_future_callback(self, future):
        """
        Callback function for the future object

        Args:
            future (Future): A future object
        """
        self.get_logger().info(Color.GREEN + 'Submission complete' + Color.RESET)
        

    def complete_order_publish_message(self):
        """Function to check order completion
        
        This function is used to check if all the orders have been completed"
        
        """
        # checking order completion condition
        msg = Bool()
        msg.data = (len(self._submitted_orders) == self._total_orders) and (len(self._submitted_orders) > 0)
        # publishing the boolean object
        self._complete_order_pub.publish(msg)