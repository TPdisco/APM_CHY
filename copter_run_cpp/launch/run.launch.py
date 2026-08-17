"""
无人机运动控制 Launch 文件

流程:
  1. 切换到 GUIDED 模式
  2. 解锁电机
  3. 发布零速度命令 → 触发 VelAccel 子模式
  4. 持续发布前进速度命令

用法:
  ros2 launch copter_run_cpp run.launch.py forward_speed:=0.5
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    forward_speed = LaunchConfiguration('forward_speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'forward_speed',
            default_value='0.5',
            description='无人机前进速度 (m/s)'),

        Node(
            package='copter_run_cpp',
            executable='copter_run_cpp_node',
            name='copter_run_node',
            output='screen',
            parameters=[{
                'forward_speed': forward_speed,
                'publish_rate': 30.0,
            }],
        ),
    ])