# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# This function is needed
def generate_launch_description():
    """
    The function will help with launching the of the nodes at the same time.
    """
    # Launch descriptor object. 
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Python node check_competition_state_py.
    check_competition_state_py = Node(
        package="rwa3_group_2",
        executable="check_competition_state.py",
        parameters=[
                    {"use_sim_time": use_sim_time},
                    ],    # parameter file
    )
    
    # Python node orders_py.
    orders_py = Node(
        package="rwa3_group_2",
        executable="orders.py",
        parameters=[
                    {"use_sim_time": use_sim_time},
                    ],    # parameter file
    )
    

    

    # Python node ship_orders_py.
    ship_orders_py = Node(
        package="rwa3_group_2",
        executable="ship_orders.py",
        parameters=[
                    {"use_sim_time": use_sim_time},
                    ],    # parameter file
    )
    
    
    # Python node submit_order_py.
    submit_order_py = Node(
        package="rwa3_group_2",
        executable="submit_order.py",
    )

    # Adding the nodes to Launch descriptor object.
    #ld.add_action(cmd_line_parameter)
    ld.add_action(check_competition_state_py)
    ld.add_action(orders_py)
    ld.add_action(submit_order_py)
    ld.add_action(ship_orders_py)
    return ld
