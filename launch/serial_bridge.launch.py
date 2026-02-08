from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('hamals_serial_bridge')

    config_file = os.path.join(
        pkg_share,
        'config',
        'serial_bridge.yaml'
    )

    return LaunchDescription([

        # --------------------
        # Launch Arguments
        # --------------------
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to serial bridge config file'
        ),

        # --------------------
        # Serial Bridge Node
        # --------------------
        Node(
            package='hamals_serial_bridge',
            executable='serial_node',
            name='hamals_serial_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config')
            ]
        ),
    ])