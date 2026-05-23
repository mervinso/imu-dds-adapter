from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_dds_adapter'),
        'config', 'adapter_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='imu_dds_adapter',
            executable='imu_dds_adapter_node',
            name='imu_dds_adapter',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
