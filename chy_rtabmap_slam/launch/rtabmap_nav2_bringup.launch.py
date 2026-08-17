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
    # 包路径
    pkg_share = get_package_share_directory('chy_rtabmap_slam')
    default_map_yaml = os.path.join(pkg_share, 'map', 'my_map3.yaml')

    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml = LaunchConfiguration('map_yaml')

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
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='静态地图yaml文件路径'),


        # 2. map_server：发布静态地图（用于全局路径规划）
        # 使用 map_yaml 参数指定地图文件路径
        # 注意：Nav2 static_layer 需要 Transient Local QoS
        # 如果不需要地图（纯视觉导航），可以注释掉此节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,  # 使用 launch 参数
                'topic_name': '/map',
                'frame_id': 'map',
                # Nav2 static_layer 需要以下 QoS 设置
                'map_subscribe_transient_local': True,
                # 确保地图在启动时发布
                'save_map_timeout': 5000.0,
            }],
        ),
        
        # 2. 点云到 2D 激光扫描转换（用于局部避障）
        Node(
            package='pointcloud_to_laserscan',         # ✅ 正确包名
            executable='pointcloud_to_laserscan_node', # ✅ 正确可执行文件名
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.3,
                'max_height': 2.0,
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

        # ======================== 替代 localization.launch.py 的节点 ========================
        # 以下节点提供仿真环境中的定位和地图服务，无需 rtabmap

        # 1. 静态 TF: map → odom（identity transform，仿真环境中重合）
        # 在真实无人机上，这应该由 rtabmap 或 AMCL 提供
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),


        # 3. lifecycle_manager：自动激活 map_server
        # Nav2 的 map_server 是 lifecycle 节点，需要手动激活
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 10.0,
                'attempt_respawn_reconnection': True,
            }],
        ),

        # 3. stereo_image_proc：生成视差图（用于深度信息）
        # 从双目图像生成视差图 → /stereo/disparity
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',
            namespace='stereo',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('left/image_rect', '/stereo/left/image_raw'),
                ('right/image_rect', '/stereo/right/image_raw'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
            ],
        ),

    ])