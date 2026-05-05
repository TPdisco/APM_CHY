# stereo_minimal.launch.py
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node,SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    # rtabmap_odom (stereo_odometry) - 标准参数
    odom_standard = {
        # ========== 坐标系 ==========
        'frame_id': 'base_link',      # 相机坐标系
        'odom_frame_id': 'odom',             # 里程计父坐标系
        'child_frame_id': 'base_link',  # 机器人本体（建议用base_footprint而非base_link）
        
        # ========== 订阅设置 ==========
        'subscribe_stereo': True,
        # 'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        
        # ========== IMU 核心设置 ==========
        'wait_imu_to_init': True,           # 等待 IMU 初始化（关键！）
        # 'use_imu': True,                    # 启用 IMU
        # 'imu_topic': '/imu',           # IMU 话题
        # 'imu_frame_id': 'iris/imu_link/imu_sensor',         # IMU 坐标系（必须与 TF 中的 child_frame 匹配）
        # 'always_check_imu_tf': True,        # 始终检查IMU TF
        # 'Imu/BufferSize': '100',             # IMU 内部缓冲大小
        # 'Imu/MaxInterval': '0.1',            # IMU 最大插值间隔

        # # ========== IMU 预处理 ==========
        # 'Imu/FilterStrategy': '1',          # 0=无滤波, 1=卡尔曼, 2=Madgwick
        # 'Imu/GravitySigma': '0.0',          # 重力加速度标准差(0=自动估计)

        # # ========== 时间设置 ==========
        # 'use_sim_time': True,
        # 'approx_sync': True,
        # 'approx_sync_max_interval': 0.05,    # 10ms同步窗口
        # 'sync_queue_size': 20,                    # 输入队列长度
        # 'sync_tolerance': '0.05',             # 同步容差 10ms
        # 'topic_queue_size': 100,
        
        # # ========== 估计器选择 ==========
        # # 0=视觉+IMU (OKVIS风格), 1=纯视觉PnP, 2=仅IMU
        # 'Vis/EstimationType': '0',
        
        # # ========== 视觉-IMU融合权重 ==========
        # 'Vis/ImuGravity': '9.81',           # 重力加速度值
        # 'Vis/ImuBiasModel': '0',            # 0=常数偏置, 1=随机游走

        # # ========== TF设置 ==========
        # 'publish_tf': True,
        # 'wait_for_transform': 0.2,           # TF等待超时
        # 'publish_frequency': 30.0,           # TF发布频率
        
        # ========== 特征检测（性能关键） ==========
        # 'Kp/MaxFeatures': '300',             # 每帧特征点数
        'Kp/DetectorStrategy': '2',          # 2=ORB（快速）
        'Vis/FeatureType': '2',              # 2=ORB描述子
        'Vis/CorType': '0',                 # 0=词袋匹配, 1=光学流

        # "Vis/MaxDepth": "8.0",            # 最大深度4米（滤除远处特征，提升精度）
        # "Vis/CorNNDR": "0.8",           # 最近邻比率0.7（提升匹配质量）默认0.8
        # "Vis/MinDepth": "0.1",            # 最小深度0.3米（滤除近处噪声）
        
        # # ========== 运动模型 ==========
        # 'Odom/Strategy': '0',               # 0=帧到帧, 1=帧到地图

        # # ========== 立体匹配 ==========
        # 'Stereo/MaxDisparity': '256.0',      # 最大视差
        # 'Stereo/MinDisparity': '0',
        
        # # ========== 初始化与跟踪 ==========
        # 'Vis/MinInliers': '8',              # PnP最小内点
        # 'Odom/MinInliers': '8',
        
        # # ========== 性能保护 ==========
        # 'Rtabmap/TimeThr': '0.4',            # 单帧最大处理时间200ms

        # # ========== 新增：丢失恢复机制 ==========
        # 'Odom/ResetCountdown': '1',          # 【新增】丢失后立即重置，快速恢复
        # 'Vis/MaxFeatures': '600',            # 【新增】备用特征数上限
    }

    # rtabmap_slam (rtabmap) - 标准参数
    slam_standard = {
        # ========== 坐标系 ==========
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        
        # ========== 订阅设置 ==========
        'subscribe_stereo': True,
        # 'subscribe_rgbd': True,
        'subscribe_odom_info': True,        # 订阅里程计信息（由odom节点提供）

        # ========== 时间设置 ==========
        # 'use_sim_time': True,
        # 'approx_sync': True,
        
        # ========== 回环检测 ==========
        # 'Rtabmap/DetectionRate': '1',        # 处理频率2Hz
        # 'Rtabmap/LoopThr': '0.1',           # 回环阈值（越小越宽松）
        
        # ========== 特征与更新 ==========
        # 'Kp/MaxFeatures': '300',             # SLAM特征数可略高于odom
        'RGBD/LinearUpdate': '0',         # 最小平移更新距离(5cm)
        'RGBD/AngularUpdate': '0',        # 最小旋转更新角度(0.05rad≈3°)
        
        # ========== 性能优化 ==========
        # 'Rtabmap/TimeThr': '0.3',            # 单帧最大处理时间300ms
        # 'sync_queue_size': 10,                     # 短队列降低延迟
        
        # 'Grid/CellSize': '0.1',              # 栅格分辨率10cm
        # 'Mem/ReduceGraph': 'true',           # 【新增】启用图优化简化
        
        # ========== TF设置 ==========
        # 'wait_for_transform': 0.8,
        # 'publish_tf': True,                 # SLAM不发布TF（由odom发布odom->base）

        # 'Rtabmap/StartNewMapOnLoopClosure': 'false',
        # ============================================
        # ========== OctoMap 3D 地图生成配置 ==========
        # ============================================
        
        # ---- 核心开关 ----
        'Grid/3D': 'true',                   # 启用 3D 占用网格（OctoMap 必需）
        'Grid/RayTracing': 'true',           # 启用光线追踪填充未知空间
        
        # ---- 传感器选择 ----
        # 0=激光雷达, 1=深度图像(RGB-D), 2=两者都用
        'Grid/Sensor': '1',                  # 使用深度图像生成 OctoMap
        
        # ---- 网格分辨率 ----
        'Grid/CellSize': '0.05',             # OctoMap 分辨率 5cm（建议0.05-0.1）





        # ========== 2D栅格地图配置 ==========
        # 启用2D地图
        'Grid/FromDepth': 'true',          # 从深度图生成2D网格（必需）

        # 'Grid/MaxObstacleHeight': '3',   # 最大障碍物高度（米）
        # 'Grid/MinGroundHeight': '-0.1',    # 地面以下的最小高度
        # 'Grid/NormalsSegmentation': 'true', # 使用法向量分割地面
        # 'Grid/MaxGroundAngle': '30',       # 地面最大角度（度）
        # 'Grid/RangeMax': '5.0',           # 最大探测范围（米）
        # 'Grid/RangeMin': '0.2',            # 最小探测范围（米）
        
        # 2D地图发布设置
        'grid_map': 'true',                # 启用2D栅格地图发布
        'grid_unknown_space': 'true',      # 在地图中包含未知区域
        'grid_cell_size': '0.05',          # 与Grid/CellSize保持一致
        'grid_size': '50',                 # 地图大小（米），0=自动调整
        'grid_incremental': 'false',       # 设为false使用全局地图
        
        # 占用阈值
        'GridGlobal/OccupancyThr': '0.5',  # 占用概率阈值（0-1）
        'GridGlobal/ProbHit': '0.7',       # 命中概率
        'GridGlobal/ProbMiss': '0.4',      # 未命中概率
        
        # ========== 内存管理优化用于2D地图 ==========
        'Mem/IncrementalMemory': 'true',
        # 'Mem/STMSize': '30',
        'Mem/RehearsalSimilarity': '0.1',
        
        # 确保地图数据被保存和发布
        'publish_map_data': 'true',
        'map_cloud_output': 'true',



        # ========== 新增：鲁棒性增强 ==========
        # 'Rtabmap/MaxRetrieved': '2',         # 【新增】限制检索数量，加速回环检测
        # 'RGBD/ProximityPathMaxNeighbors': '10', # 【新增】限制邻近搜索范围
        # 'RGBD/ProximityBySpace': 'true',     # 【新增】启用空间邻近检测
        # 'RGBD/ProximityByTime': 'false',     # 【新增】关闭时间邻近，避免错误关联
        # 'Vis/CorNNDR': '0.8',                # 【新增】匹配阈值与odom一致
    }



    remappings=[
        #   ('left/image_rect', '/stereo/left/image_raw_fix'),
        #   ('left/camera_info', '/stereo/left/camera_info_fix'),
        #   ('right/image_rect', '/stereo/right/image_raw_fix'),
        #   ('right/camera_info', '/stereo/right/camera_info_fix'),
        ('left/image_rect', '/stereo_camera/left/image_rect'),
        ('right/image_rect', '/stereo_camera/right/image_rect'),
        ('left/camera_info', '/stereo_camera/left/camera_info'),
        ('right/camera_info', '/stereo_camera/right/camera_info'),
            # ('rgbd_image', '/stereo_camera/rgbd_image'),
        #   ('imu', '/imu'), 
            ('imu', '/imu/data'), 
        ]


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),
        
        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node. If the same argument is already set in \"args\", it will be overwritten by the one in \"odom_args\".'),


        # Node(
        #     package='chy_rtabmap_slam',
        #     executable='map_pub', 
        #     name='pub_map_fixed',
        # ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_camera_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_to_imu_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'iris/imu_link/imu_sensor'],
        #     parameters=[{'use_sim_time': True}]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_to_left_camera_tf',
        #     arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'iris/camera_link/left_camera'],
        #     parameters=[{'use_sim_time': True}]
        # ),

        # Node(
        #     package='rtabmap_util', executable='yaml_to_camera_info.py', output='screen',
        #     parameters=[{'yaml_path': [FindPackageShare('chy_rtabmap_slam'), '/config/euroc_left.yaml']}],
        #     remappings=[
        #       ('image', '/cam0/image_raw'),
        #       ('camera_info', 'left/camera_info')],
        #     namespace='stereo_camera'),
              
        # Node(
        #     package='rtabmap_util', executable='yaml_to_camera_info.py', output='screen',
        #     parameters=[{'yaml_path': [FindPackageShare('chy_rtabmap_slam'), '/config/euroc_right.yaml']}],
        #     remappings=[
        #       ('image', '/cam1/image_raw'),
        #       ('camera_info', 'right/camera_info')],
        #     namespace='stereo_camera'),

        Node(
            package='image_proc', executable='rectify_node', output='screen',
            remappings=[
              ('image', '/cam0/image_raw')],
            namespace='stereo_camera/left'),
        Node(
            package='image_proc', executable='rectify_node', output='screen',
            remappings=[
              ('image', '/cam1/image_raw')],
            namespace='stereo_camera/right'),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '3.1415926', '-1.570796', '0', 'base_link', 'imu4']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.021640', '-0.064677', '0.009811', '1.555925', '0.025777', '0.003757', 'imu4', 'cam0']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['-0.019844', '0.045369', '0.007862', '1.558237', '0.025393', '0.017907', 'imu4', 'cam1']),
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),

        Node(
        package='imu_complementary_filter', executable='complementary_filter_node', output='screen',
        parameters=[{'use_mag': False, 'world_frame':'enu', 'publish_tf':False}],
        remappings=[('imu/data_raw', '/imu0')]),

        # Node(
        #     package='rtabmap_sync', executable='stereo_sync', output='screen',
        #     namespace='stereo_camera',
        #     parameters=[{
        #         'sync_queue_size': 30,
        #         }],
        #         remappings=[
        #             # ('left/image_rect', '/stereo/left/image_raw'),
        #             # ('right/image_rect', '/stereo/right/image_raw'),
        #             # ('left/camera_info', '/stereo/left/camera_info'),
        #             # ('right/camera_info', '/stereo/right/camera_info'),
                    # ('left/image_rect', '/stereo_camera/left/image_rect'),
                    # ('right/image_rect', '/stereo_camera/right/image_rect'),
                    # ('left/camera_info', '/stereo_camera/left/camera_info'),
                    # ('right/camera_info', '/stereo_camera/right/camera_info'),
        #         ]
        #     ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[odom_standard],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"),
                        '--ros-args', '--log-level', 'info'],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_standard],
            remappings=remappings,
            arguments=['-d', LaunchConfiguration("args"),
                    '--ros-args', '--log-level', 'info']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[slam_standard,
                        {'odometry_node_name': "stereo_odometry"}],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'info'])
                
    ])