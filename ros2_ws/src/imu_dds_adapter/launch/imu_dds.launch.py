from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('imu_dds_adapter')
    config = os.path.join(pkg, 'config', 'adapter_params.yaml')
    rviz_config = os.path.join(pkg, 'config', 'imu_rviz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 visualization'),
        Node(
            package='imu_dds_adapter',
            executable='imu_dds_adapter_node',
            name='imu_dds_adapter',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
