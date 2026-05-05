# launch/apm_nav2_bridge_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('map_origin_lat', default_value='40.08370'),
        DeclareLaunchArgument('map_origin_lon', default_value='-105.21740'),
        DeclareLaunchArgument('map_origin_alt', default_value='1630.0'),
        DeclareLaunchArgument('goal_altitude', default_value='20.0'),
        DeclareLaunchArgument('auto_guided_mode', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('max_goal_distance', default_value='100.0'),
        
        # 桥接节点
        Node(
            package='chy_rtabmap_slam',
            executable='goal_converter_node',
            name='apm_nav2_bridge',
            output='screen',
            parameters=[{
                'map_origin_lat': LaunchConfiguration('map_origin_lat'),
                'map_origin_lon': LaunchConfiguration('map_origin_lon'),
                'map_origin_alt': LaunchConfiguration('map_origin_alt'),
                'goal_altitude': LaunchConfiguration('goal_altitude'),
                'auto_guided_mode': LaunchConfiguration('auto_guided_mode'),
                'auto_arm': LaunchConfiguration('auto_arm'),
                'max_goal_distance': LaunchConfiguration('max_goal_distance'),
            }]
        ),
    ])