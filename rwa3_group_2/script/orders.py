#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024
- ros2 run rwa3_group_2 orders.py
'''

import rclpy
from rwa3_group_2.orders_sub_interface import OrderSubInterface


def main(args=None):
    """
    Main function to initialize the ROS 2 node and keep it alive until it's manually terminated or receives a shutdown signal.

    Args:
        args (list, optional): Command-line arguments that can be passed to the ROS 2 node. Defaults to None.

    This function performs the following steps:
    1. Initializes the ROS 2 Python client library (rclpy) with any provided arguments.
    2. Creates an instance of `OrderSubInterface`, which subscribes to a specific topic.
    3. Spins (i.e., continuously processes messages on all subscribers) the node to keep it alive.
    4. Destroys the node and shuts down the ROS 2 system once the node stops spinning.
    """
    rclpy.init(args=args) # Initialize the ROS client library
    node = OrderSubInterface() # Create an instance of the OrderSubInterface
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().error("KeyboardInterrupt received!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() # Execute the main function when the script is run