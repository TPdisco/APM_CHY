"""
USB摄像头持续录制 Launch 文件

用法:
  ros2 launch copter_run_cpp camera_record.launch.py
  ros2 launch copter_run_cpp camera_record.launch.py camera_id:=1 output_dir:=/home/user/videos
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='USB摄像头设备ID (/dev/video0 → 0)'),
        DeclareLaunchArgument(
            'output_dir', default_value='~/camera_recordings',
            description='视频保存目录 (默认 ~/camera_recordings)'),
        DeclareLaunchArgument(
            'fps', default_value='30.0',
            description='录制帧率'),
        DeclareLaunchArgument(
            'width', default_value='640',
            description='分辨率宽度'),
        DeclareLaunchArgument(
            'height', default_value='480',
            description='分辨率高度'),

        Node(
            package='copter_run_cpp',
            executable='camera_recorder.py',
            name='camera_recorder',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'output_dir': LaunchConfiguration('output_dir'),
                'fps': LaunchConfiguration('fps'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
            }],
        ),
    ])