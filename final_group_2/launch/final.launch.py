import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    ccs_cpp = Node(
        package="final_group_2",
        executable="competition_control_system_exe",
        output="screen",
        parameters=generate_parameters(),
    )


   

    nodes_to_start = [
        ccs_cpp
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
