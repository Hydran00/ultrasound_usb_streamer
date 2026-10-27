from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ultrasound_usb"), "config", "ultrasound.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=config),
            DeclareLaunchArgument(
                "device", default_value="",
                description="V4L2 device; empty keeps the value from params_file",
            ),
            DeclareLaunchArgument(
                "preview",
                default_value="true",
                description="Open the video preview at 720 pixels high",
            ),
            OpaqueFunction(function=launch_node),
        ]
    )


def launch_node(context):
    overrides = {
        "preview": ParameterValue(LaunchConfiguration("preview"), value_type=bool),
    }
    device = LaunchConfiguration("device").perform(context)
    if device:
        overrides["device"] = ParameterValue(device, value_type=str)
    return [
        Node(
            package="ultrasound_usb",
            executable="ultrasound_usb_node",
            name="ultrasound_usb",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[LaunchConfiguration("params_file"), overrides],
        )
    ]
