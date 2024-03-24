
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
    AGVControlClient is a node that listens to orders and processes them by locking the tray and moving the AGV.

    """
    def __init__(self):
        super().__init__('agv_control_client')
        # self.subscription = self.create_subscription(Order, '/ariac/orders', self.order_callback, 10)
        self.subscription = self.create_subscription(Int32, 'fullfilled_agv_id', self.order_callback, 10)

        self._agv_orders = [] 

    @property

    def orders(self):

        return self._agv_orders

    def order_callback(self, msg):
        """
        Callback function for the order subscription.

        """
        agv_id = msg.data
        self.get_logger().info(f'Received order for AGV {agv_id}')
        self._agv_orders.append(agv_id)
        # self.process_agv_orders()

    # def lock_tray(self, agv_id):
    #     """
    #     Locks the tray of the AGV.
    #     wait_for_service() is used to wait for the service to become available.
    #     call_async() is used to call the service asynchronously.
    #     spin_until_future_complete() is used to wait for the future to be completed.
    #     """
    #     client = self.create_client(Trigger, f'/ariac/agv{agv_id}_lock_tray')
    #     self.get_logger().info('Inside lock tray after creating clinet')

    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn(f'/ariac/agv{agv_id}_lock_tray service not available, waiting...')
    #     req = Trigger.Request()
    #     self.get_logger().info('Inside lock tray after Triggering request')
    #     future = client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()

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
        
        # return future.result()

        # Check the response
        if future.result().success:
            self.get_logger().info(f'Locked AGV{num}\'s tray')
            return future.result()
        else:
            self.get_logger().warn('Unable to lock tray')
            return future.result()


    # def move_agv(self, agv_id, station='destination_station'):
    #     """
    #     Moves the AGV to the destination station.
    #     client.wait_for_service() is used to wait for the service to become available.
    #     call_async() is used to call the service asynchronously.
    #     spin_until_future_complete() is used to wait for the future to be completed.
    #     req.station is set to the destination station.

    #     """
    #     client = self.create_client(MoveAGV, f'/ariac/move_agv{agv_id}')
    #     self.get_logger().info('Inside MOVE AGV after creating clinet')

    #     while not client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().warn(f'/ariac/move_agv{agv_id} service not available, waiting...')
    #     req = MoveAGV.Request()
    #     self.get_logger().info('Inside MOVE AGV after Triggering request')
    #     req.station = station
    #     future = client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result()
    
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


    def process_agv_orders(self):
        """
        Processes the AGV orders by locking the tray and moving the AGV.
        The AGV orders are processed in the order of priority, with the highest priority AGV being processed first.
        priority_agv_id is set to the highest priority AGV id.
        lock_result is set to the result of the lock_tray() function.
        move_result is set to the result of the move_agv() function.
        If the lock_result is successful, the move_agv() function is called.
        If the move_result is not successful, an error message is logged.
        If the lock_result is not successful, an error message is logged.
        The AGV id is removed from the agv_orders list.
        """
        for priority_agv_id in sorted(self._agv_orders, reverse=True):
            self.get_logger().info(f'Processing AGV {priority_agv_id}')
            lock_result = self.lock_tray(priority_agv_id)
            if lock_result.success:
                self.get_logger().info(f'Success to lock tray for AGV {priority_agv_id}')
                move_result = self.move_agv(priority_agv_id)
                if not move_result.success:
                    self.get_logger().error(f'Failed to move AGV {priority_agv_id}')
                else:
                    self.get_logger().info(f'Success to move AGV {priority_agv_id}')
            else:
                self.get_logger().error(f'Failed to lock tray for AGV {priority_agv_id}')
            self._agv_orders.remove(priority_agv_id)

# def main(args=None):
#     rclpy.init(args=args)
#     agv_control_client = AGVControlClient()
#     rclpy.spin(agv_control_client) 
#     agv_control_client.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()    
