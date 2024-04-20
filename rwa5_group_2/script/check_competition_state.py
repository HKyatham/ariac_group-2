#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4_spring2024
- ros2 run rwa5_group_2 check_competition_state.py
'''

import rclpy
from rwa5_group_2.check_competition_state_interface import CheckCompetitionStateInterface


def main(args=None):
    """
    Main function to initialize the ROS 2 node and keep it alive until it's manually terminated or receives a shutdown signal.

    Args:
        args (list, optional): Command-line arguments that can be passed to the ROS 2 node. Defaults to None.

    This function performs the following steps:
    1. Initializes the ROS 2 Python client library (rclpy) with any provided arguments.
    2. Creates an instance of `CheckCompetitionStateInterface`, which subscribes to a specific topic.
    3. Spins (i.e., continuously processes messages on all subscribers) the node to keep it alive.
    4. Destroys the node and shuts down the ROS 2 system once the node stops spinning.
    """
    rclpy.init(args=args) # Initialize the ROS client library
    competitionNode = CheckCompetitionStateInterface() # Create an instance of the CheckCompetitionStateInterface
    try:
        competitionNode.get_logger().info("Inside try block.")
        competitionNode.start_competition()
       

        # end the competition if all orders have been submitted
        # and the competition state is ORDER_ANNOUNCEMENTS_DONE
        competitionNode.end_competition()
    except KeyboardInterrupt:
        competitionNode.get_logger().error("KeyboardInterrupt received!")
    finally:
        competitionNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() # Execute the main function when the script is run