from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='razor_imu',
            namespace='razor_imu1',
            executable='imuStream',
            name='imu_stream',
        )
    ])
