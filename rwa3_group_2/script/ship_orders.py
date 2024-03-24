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
    #     rclpy.spin(agv_control_client)  
        # while not agv_control_client.orders:
            while True:
                rclpy.spin_once(agv_control_client)
                agv_control_client.get_logger().info(f'INSIDE SPIN')

                if (agv_control_client.orders):
                    for agv in sorted(agv_control_client.orders):
                        agv_control_client.get_logger().info(f'Processing AGV {agv}')
                        lock_result=agv_control_client.lock_tray(agv)
                        if lock_result.success:
                            agv_control_client.get_logger().info(f'Success to lock tray for AGV {agv}')
                            move_result=agv_control_client.move_agv(agv,3)
                            if not move_result.success:
                                agv_control_client.get_logger().error(f'Failed to move AGV {agv}')
                            else:
                                agv_control_client.get_logger().info(f'Success to move AGV {agv}')
                        else:
                            agv_control_client.get_logger().error(f'Failed to lock tray for AGV {agv}')
                        agv_control_client.orders.remove(agv)
            # except KeyboardInterrupt:
            #     break

    except KeyboardInterrupt:
        agv_control_client.get_logger().error("KeyboardInterrupt received!")
    finally:
        agv_control_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 