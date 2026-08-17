# launch/underwater_visual_degrader.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'left_input_topic',
            default_value='/stereo/left/image_raw',
            description='左目原始图像话题'),
        DeclareLaunchArgument(
            'left_output_topic',
            default_value='/stereo/left/underwater',
            description='左目退化后图像输出话题'),
        DeclareLaunchArgument(
            'right_input_topic',
            default_value='/stereo/right/image_raw',
            description='右目原始图像话题'),
        DeclareLaunchArgument(
            'right_output_topic',
            default_value='/stereo/right/underwater',
            description='右目退化后图像输出话题'),
        DeclareLaunchArgument(
            'turbidity',
            default_value='0.3',
            description='浑浊度 [0, 1]，越大越模糊'),
        DeclareLaunchArgument(
            'depth',
            default_value='0.5',
            description='深度因子 [0, 1]，控制颜色吸收和光照衰减'),
        DeclareLaunchArgument(
            'backscatter',
            default_value='0.1',
            description='后向散射强度 [0, 1]，越大噪声越多'),
        DeclareLaunchArgument(
            'enable',
            default_value='true',
            description='是否启用退化效果'),

        # 水下视觉退化节点
        Node(
            package='chy_rtabmap_slam',
            executable='underwater_visual_degrader',
            name='underwater_visual_degrader',
            output='screen',
            parameters=[{
                'left_input_topic': LaunchConfiguration('left_input_topic'),
                'left_output_topic': LaunchConfiguration('left_output_topic'),
                'right_input_topic': LaunchConfiguration('right_input_topic'),
                'right_output_topic': LaunchConfiguration('right_output_topic'),
                'turbidity': LaunchConfiguration('turbidity'),
                'depth': LaunchConfiguration('depth'),
                'backscatter': LaunchConfiguration('backscatter'),
                'enable': LaunchConfiguration('enable'),
            }],
        ),
    ])