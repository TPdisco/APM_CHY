# launch/stereo_inertial_nav2_bringup.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # 包路径
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')
    # chy_dir = get_package_share_directory('chy_rtabmap_slam')
    #     # 自定义行为树路径
    # bt_xml_path = os.path.join(chy_dir, 'behavior_trees', 'nav_to_pose_simple.xml')
    
    # # 确保文件存在
    # if not os.path.exists(bt_xml_path):
    #     raise FileNotFoundError(f"Behavior tree file not found: {bt_xml_path}")
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        
        # 2. 点云到 2D 激光扫描转换（用于局部避障）
        Node(
            package='pointcloud_to_laserscan',         # ✅ 正确包名
            executable='pointcloud_to_laserscan_node', # ✅ 正确可执行文件名
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.05,
                'max_height': 1.5,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.0333,
                'range_min': 0.3,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud_in', '/cloud_map'),  # 输入点云
                ('scan', '/scan')                     # 输出激光扫描
            ]
        ),
        
        # 3. Nav2 导航栈
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'bringup_launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('chy_rtabmap_slam'), 'config', 'nav2_params_2.yaml'
                ]),
                'autostart': 'true',
                'map': '',  # 使用 RTAB-Map 动态地图
            }.items()
        ),

        Node(
            package='chy_rtabmap_slam',
            executable='vel_converter_node',
            name='apm_nav2_bridge',
            output='screen',
        ),

    ])