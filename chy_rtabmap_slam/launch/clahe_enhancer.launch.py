# launch/clahe_enhancer.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_input_topic',
            default_value='/stereo/left/underwater',
            description='左目输入图像话题'),
        DeclareLaunchArgument(
            'left_output_topic',
            default_value='/stereo/left/enhanced',
            description='左目 CLAHE 增强后输出话题'),
        DeclareLaunchArgument(
            'right_input_topic',
            default_value='/stereo/right/underwater',
            description='右目输入图像话题'),
        DeclareLaunchArgument(
            'right_output_topic',
            default_value='/stereo/right/enhanced',
            description='右目 CLAHE 增强后输出话题'),
        DeclareLaunchArgument(
            'clip_limit',
            default_value='2.0',
            description='对比度限制阈值 [0.5, 5.0]，越大对比度越强'),
        DeclareLaunchArgument(
            'tile_size',
            default_value='8',
            description='分块大小 [4, 32]，越小局部细节越强'),
        DeclareLaunchArgument(
            'lab_mode',
            default_value='true',
            description='true=仅在Lab亮度通道做CLAHE(保色), false=RGB三通道独立做'),
        DeclareLaunchArgument(
            'enable',
            default_value='true',
            description='是否启用 CLAHE 增强'),

        Node(
            package='chy_rtabmap_slam',
            executable='clahe_enhancer',
            name='clahe_enhancer',
            output='screen',
            parameters=[{
                'left_input_topic': LaunchConfiguration('left_input_topic'),
                'left_output_topic': LaunchConfiguration('left_output_topic'),
                'right_input_topic': LaunchConfiguration('right_input_topic'),
                'right_output_topic': LaunchConfiguration('right_output_topic'),
                'clip_limit': LaunchConfiguration('clip_limit'),
                'tile_size': LaunchConfiguration('tile_size'),
                'lab_mode': LaunchConfiguration('lab_mode'),
                'enable': LaunchConfiguration('enable'),
            }],
        ),
    ])