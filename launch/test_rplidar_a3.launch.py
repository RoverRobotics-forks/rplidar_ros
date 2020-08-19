from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidarNode',
            package='rplidar_ros',
            executable='rplidarNode',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
        ),
        Node(
            name='rplidarNodeClient',
            package='rplidar_ros',
            executable='rplidarNodeClient',
            output='screen',
        ),
    ])
