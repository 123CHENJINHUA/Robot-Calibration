from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start RealSense driver
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            output='screen',
            parameters=[{
                # Common defaults
                'enable_color': True,
                'rgb_camera.color_profile': '640x480x30',
                # If you have multiple cameras, set serial_no
                # 'serial_no': 'XXXXXXXXXXXX',
            }],
        ),

        # ChArUco logger
        Node(
            package='handeye_pkg',
            executable='handeye_node',
            name='handeye_node',
            output='screen',
        ),
    ])
