import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32MultiArray
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
    It logs the received messages to the ROS2 logger. This class also has a service client "/ariac/submit_order" service to submit order.
    
    Attributes:
        _submit_order_client (rclpy.client): The client object for calling the service.
        _agv_status_sub (rclpy.subscription.Subscription): The subscription object for receiving AGV status messages.
        _agv_status (ariac_msgs/msg/AGVStatus): A Int to store state values from /ariac/agv{n}_status topic.
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """    
    mutex_group1 = MutuallyExclusiveCallbackGroup()
    reentrant_group1 = ReentrantCallbackGroup()

    
 
    def __init__(self):
        super().__init__('OrderSubmissionInterface')
        
        self.get_logger().info('Order Submission node started!')
        
        self._orders_sub = self.create_subscription(Order,'/ariac/orders',self.orders,10,callback_group= OrderSubmissionInterface.mutex_group1)
        self. _submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order')
        self._submitted_orders = set() 

        self._complete_order_pub = self.create_publisher(Bool, "/complete_order", 1)
        self._complete_order_timer = self.create_timer(1, self.complete_order_publish_message)

        self._agv_1_status_sub = self.create_subscription(AGVStatus, '/ariac/agv1_status', self.agv_1_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_2_status_sub = self.create_subscription(AGVStatus, '/ariac/agv2_status', self.agv_2_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_3_status_sub = self.create_subscription(AGVStatus, '/ariac/agv3_status', self.agv_3_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        self._agv_4_status_sub = self.create_subscription(AGVStatus, '/ariac/agv4_status', self.agv_4_status_cb, 10,callback_group= OrderSubmissionInterface.reentrant_group1)
        
        self._id_agv_dict = {}
        self._id_1_submit = None
        self._id_2_submit = None
        self._id_3_submit  = None
        self._id_4_submit = None
        self._submit_request = SubmitOrder.Request()
        self._agv_location_status = AGVStatus()
        self._order_id = None
        self._agv_number = None
        self._order_id_list = set()
        self._total_orders = 0
        self._complete_ordre = bool()
    
    def orders(self, msg):
        """Handle incoming messages on the "/ariac/orders" topic to link AGV number and order id.

        This function is called when a new message is received on the "/ariac/orders" topic. It stores the order id and agv number.

        Args:
            msg (ariac.msgs.msg.Order): The received message object, containing the CompetitionState data.
        """
        self.get_logger().info(f'Order ID is >>>>>>>?///////....... : {msg.id}')
        self._order_id= msg.id
        self._agv_number = msg.kitting_task.agv_number
        self._id_agv_dict[self._agv_number] = self._order_id
        self.get_logger().info(f'Order ID is : {self._order_id}')
        self._total_orders += 1
        self.get_logger().info(f'Total Order Count : {self._total_orders}')
        self._complete_ordre = False
       
        
        
    def agv_1_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # self.get_logger().info('Inside agv-1 cb')
        self._agv_1_location_status = msg.location
        # self.get_logger().info(f'AGV-1 location is : {self._agv_1_location_status}')

        if self._agv_1_location_status == AGVStatus.WAREHOUSE:
            self._id_1_submit = self._id_agv_dict.get(1)
            # self.get_logger().info(f'Order ID inside warehouse is .............: {self._id_1_submit}')
            if self._id_1_submit not in self._submitted_orders:
                self._submitted_orders.add(self._id_1_submit)
                self._order_id_list.add(self._id_1_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_1_submit}')
                self._submit_order_request(self._id_1_submit)
                self.get_logger().info('The AGV-1 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def agv_2_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # self.get_logger().info('Inside agv-2 cb')
        self._agv_2_location_status = msg.location
        # self.get_logger().(f'AGV-2  location is : {self._agv_2_location_status}')
        
        if self._agv_2_location_status == AGVStatus.WAREHOUSE:
            self._id_2_submit = self._id_agv_dict.get(2)
            if self._id_2_submit not in self._submitted_orders:
                self._submitted_orders.add(self._id_2_submit)
                self._order_id_list.add(self._id_2_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_2_submit}')
                self._submit_order_request(self._id_2_submit)
                self.get_logger().info('The AGV-2 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def agv_3_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # self.get_logger().info('Inside agv-3 cb')
        self._agv_3_location_status = msg.location
        # self.get_logger().info(f'AGV-3  location is : {self._agv_3_location_status}')
        
        
        if self._agv_3_location_status == AGVStatus.WAREHOUSE:
            self._id_3_submit = self._id_agv_dict.get(3)
            if self._id_3_submit not in self._submitted_orders:
                self._submitted_orders.add(self._id_3_submit)
                self._order_id_list.add(self._id_3_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_3_submit}')
                self._submit_order_request(self._id_3_submit)
                self.get_logger().info('The AGV-3 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
    def agv_4_status_cb(self,msg):
        """Handle incoming messages on the "/ariac/agv{n}_status " topic.

        This function is called when a new message is received on the "/ariac/agv{n}_status topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.AGVStatus): The received message object, containing the AGVStatus data.
        """
        # self.get_logger().info('Inside agv-4 cb')
        self._agv_4_location_status = msg.location
        # self.get_logger().info(f'AGV-4 location is : {self._agv_4_location_status}')
        self._id_4_submit = self._id_agv_dict.get(4)
        

        if self._agv_4_location_status == AGVStatus.WAREHOUSE:
            self._id_4_submit = self._id_agv_dict.get(4)
            if self._id_4_submit not in self._submitted_orders:
                self._submitted_orders.add(self._id_4_submit)
                self._order_id_list.add(self._id_4_submit)
                self.get_logger().info(f'Order ID for submission is: {self._id_4_submit}')
                self._submit_order_request(self._id_4_submit)
                self.get_logger().info('The AGV-4 has reached ' + Color.GREEN + 'WAREHOUSE' + Color.RESET)
            
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
        # self._submit_order_client.call_async(self._submit_request)
        future = self._submit_order_client.call_async(self._submit_request)
        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.submit_future_callback)

    def submit_future_callback(self, future):
        """
        Callback function for the future object

        Args:
            future (Future): A future object
        """
        self.get_logger().info(f"Submit complete.")

    def complete_order_publish_message(self):
        "This function is used to check if all the orders have been completed" 
        msg = Bool()
        msg.data = (len(self._submitted_orders) == self._total_orders) and (len(self._submitted_orders) > 0)
        self.get_logger().info(f'Publishing to /complete_orders: {msg.data}')
        self._complete_order_pub.publish(msg)