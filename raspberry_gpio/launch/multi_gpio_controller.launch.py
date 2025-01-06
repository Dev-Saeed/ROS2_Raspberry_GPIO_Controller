from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('raspberry_gpio'),
        'config',
        'multi_gpio_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='raspberry_gpio',
            executable='multi_gpio_control_node',
            name='multi_gpio_control_node',
            parameters=[config_path],
            output='screen',
        )
    ])
