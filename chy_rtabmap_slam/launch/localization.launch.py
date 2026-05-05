# ============================================================================
# 文件名: localization.launch.py
# 描述: rtabmap纯定位模式launch文件（RL训练专用）
#
# 架构设计（静态地图 + 纯定位）：
#   - map_server: 发布预构建的静态地图到 /map 话题
#   - stereo_image_proc: 从双目图像生成视差图 → /stereo/disparity
#   - rtabmap_odom (stereo_odometry): 提供视觉-惯导里程计 odom→base_link TF
#   - rtabmap_slam (rtabmap): 纯定位模式，仅提供定位
#     * 发布 odom→map TF（定位）
#     * 不发布 /map 话题（地图由 map_server 提供）
#   - RL训练: 使用 /map + /stereo/disparity + TF 进行训练
#
# 使用步骤：
#   1. 先用 slam.launch.py 建图
#   2. 保存地图: ros2 run nav2_map_server map_saver_cli -f ~/maze_map
#   3. 复制 maze_map.yaml 和 maze_map.pgm 到 map/ 目录
#   4. rtabmap.db 数据库已在 ~/.ros/rtabmap.db 中
#   5. 启动纯定位模式: ros2 launch chy_rtabmap_slam localization.launch.py
# ============================================================================

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('chy_rtabmap_slam')
    default_map_yaml = os.path.join(pkg_share, 'map', 'my_map.yaml')
    default_rtabmap_db = os.path.join(pkg_share, 'map', 'rtabmap.db')

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='静态地图yaml文件路径')

    rtabmap_db_arg = DeclareLaunchArgument(
        'rtabmap_db',
        default_value=default_rtabmap_db,
        description='rtabmap数据库路径(纯定位模式需要加载预建的数据库)')

    odom_standard = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'subscribe_rgbd': True,
        #'subscribe_stereo':True,
        'subscribe_odom_info': True,
        'wait_imu_to_init': True,
        'use_sim_time': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'sync_queue_size': 20,
        'topic_queue_size': 100,
        'Kp/DetectorStrategy': '2',
        'Vis/FeatureType': '2',
        'Vis/CorType': '0',
    }

    localization_standard = {
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'subscribe_rgbd': True,
        #'subscribe_stereo':True,
        'subscribe_odom_info': True,
        'use_sim_time': True,

        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',

        'Grid/FromDepth': 'false',
        'Grid/3D': 'false',
        'grid_map': 'false',
        'publish_map_data': 'false',

        'Grid/Resolution': '0.05',
        'Grid/MapSizeX': '50',
        'Grid/MapSizeY': '50',
        'Grid/MapOriginX': '-25',
        'Grid/MapOriginY': '-25',

        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.02',

        'Mem/RehearsalSimilarity': '0.0',

        'database_path': LaunchConfiguration('rtabmap_db'),
    }

    remappings = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        # ('left/image_rect', '/stereo/left/image_raw'),
        # ('right/image_rect', '/stereo/right/image_raw'),
        # ('left/camera_info', '/stereo/left/camera_info'),
        # ('right/camera_info', '/stereo/right/camera_info'),
        ('imu', '/imu'),
        ('odom','/odometry')
    ]

    return LaunchDescription([
        map_yaml_arg,
        rtabmap_db_arg,

        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),

        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node.'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0.075077', '0', '0', '0', 'base_link', 'camera_link'],
            # arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'iris/imu_link/imu_sensor'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_left_camera_tf',
            arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'iris/camera_link/left_camera'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            parameters=[{
                'approx_sync': True,
                'sync_queue_size': 30,
                }],
            remappings=[
                ('left/image_rect', '/stereo/left/image_raw'),
                ('right/image_rect', '/stereo/right/image_raw'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
            ]
        ),

        Node(
            package='stereo_image_proc', executable='disparity_node', output='screen',
            namespace='stereo',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('left/image_rect', '/stereo/left/image_raw'),
                ('right/image_rect', '/stereo/right/image_raw'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
            ]
        ),

        # Node(
        #     package='rtabmap_odom', executable='stereo_odometry', output='screen',
        #     parameters=[odom_standard],
        #     arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"),
        #                 '--ros-args', '--log-level', 'info'],
        #     remappings=remappings),

        Node(
            package='nav2_map_server', executable='map_server', output='screen',
            name='map_server',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': LaunchConfiguration('map_yaml'),
                
            }]),

        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'use_sim_time': True,
            }]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[localization_standard],
            remappings=remappings,
            arguments=[
                LaunchConfiguration("args"),
                '--ros-args', '--log-level', 'info']),
    ])
