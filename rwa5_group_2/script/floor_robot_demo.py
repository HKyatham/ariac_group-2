#! /usr/bin/env python3

import rclpy
from rwa5_group_2.floor_robot_interface import RobotCommanderInterface
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    '''
    Main function to create a ROS2 node

    Args:
        args (Any, optional): ROS2 arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    node = RobotCommanderInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info('Program stopped cleanly')
    except KeyboardInterrupt:
        node.get_logger().info('Program stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
