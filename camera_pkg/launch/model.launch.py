import os
from scripts.agents import agent_fbe
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = "serial_test"

    declare_model_name = DeclareLaunchArgument(
        "model_name",
        default_value="COW",
        description="The name model"
    )
    model_name = LaunchDescription("model_name")

#! HERE I GOT TO CALL THE SCRIPTS MODELS