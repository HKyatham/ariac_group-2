#!/usr/bin/env python3

import rclpy
from rwa3_group_2.ship_orders_interface import AGVControlClient


def main(args=None):
    """_
    Main function that initializes the ROS 2 node and the AGVControlClient class.
    The AGVControlClient class is then spun.
    
    """
    rclpy.init(args=args)
    agv_control_client = AGVControlClient()
    try:
        rclpy.spin(agv_control_client)  
    except KeyboardInterrupt:
        agv_control_client.get_logger().error("KeyboardInterrupt received!")
    finally:
        agv_control_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 