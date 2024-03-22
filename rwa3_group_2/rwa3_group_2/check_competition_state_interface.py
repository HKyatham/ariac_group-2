import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from ariac_msgs.msg import (
    CompetitionState,
)

from std_srvs.srv import Trigger

class Color:
    """
    A color class to modify the color of the text in logger.
    """
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    RESET = "\033[0m"

class CheckCompetitionStateInterface(Node):
    """A ROS2 CheckCompetitionStateInterface node that listens to CompetitionState messages on a specified topic and calls a service.

    This class creates a node that subscribes to the "/ariac/competition_state" topics, expecting messages of type `ariac_msgs/msg/CompetitionState`.
    It logs the received messages to the ROS2 logger. This class also has a service client to "/ariac/start_competition" service to receive a
    of type "Trigger".

    Attributes:
        _start_competition_client (rclpy.client): The client object for calling the service.
        _competition_state_sub (rclpy.subscription.Subscription): The subscription object for receiving CompetitionState messages.
        _competition_state (ariac_msgs/msg/CompetitionState): A Int to store state values from /ariac/competition_state topic.
        
    Args:
        node_name (str): The name of the node, provided during instantiation.
    """    
    def __init__(self):
        super().__init__('CheckCompetitionStateInterface')
        
        self.get_logger().info('Competition node started!')
        
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')
        
        self._competition_state_sub = self.create_subscription(CompetitionState, '/ariac/competition_state', self.competition_state_cb, 10)
        
        self._competition_state = CompetitionState()
        
        
    def competition_state_cb(self,msg):
        """Handle incoming messages on the "/ariac/competition_state" topic.

        This function is called when a new message is received on the "/ariac/competition_state" topic. It stores this data.

        Args:
            msg (ariac.msgs.msg.CompetitionState): The received message object, containing the CompetitionState data.
        """
        self.get_logger().info('Inside competition_state call back...')
        # self.get_logger().info(msg)
        
        self._competition_state = msg.competition_state
        # self.get_logger().info('Competition state is set to ' + Color.YELLOW + str(self._competition_state) + Color.RESET)
        if self._competition_state == CompetitionState.READY:
            self.get_logger().info('Competition state is set to ' + Color.YELLOW + 'READY' + Color.RESET)
            
    def start_competition(self):
        """Client request to send request to /ariac/start_competition service.
        
        This function is used to send an empty Trigger request to /ariac/start_competition service.
        
        """
        self.get_logger().info('Inside start_competition.')
        # Check if competition state is ready, if not ready spin the node.
        while self._competition_state != CompetitionState.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                self.get_logger().error('KeyboardInterrupt received!')
        # Check if the competition state is ready, if ready send the request to service.
        if self._competition_state == CompetitionState.READY:
            # Creating Request
            request = Trigger.Request()
            # Async call to the service.
            future = self._start_competition_client.call_async(request)
            
            self.get_logger().info('Inside start_competition if case.')
            # Wait until the service call is completed
            rclpy.spin_until_future_complete(self, future)
            # Check if the response from the service is success or not.
            if future.result().success:
                self.get_logger().info('Started competition.')
            else:
                 self.get_logger().warn('Unable to start competition')

    