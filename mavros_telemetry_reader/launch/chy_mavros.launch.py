#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    fcu_url = DeclareLaunchArgument('fcu_url',default_value='/dev/ttyACM0:57600',description='FCU connection string')
    gcs_url = DeclareLaunchArgument('gcs_url',default_value='',description='Ground Control Station URL')
    tgt_system = DeclareLaunchArgument('tgt_system',default_value='1',description='Target system ID')
    tgt_component = DeclareLaunchArgument('tgt_component',default_value='1',description='Target component ID')
    log_output = DeclareLaunchArgument('log_output',default_value='screen',description='Log output location')
    fcu_protocol = DeclareLaunchArgument('fcu_protocol',default_value='v2.0',description='FCU protocol version')
    respawn_mavros = DeclareLaunchArgument('respawn_mavros',default_value='false',description='Whether to respawn MAVROS on failure')
    namespace = DeclareLaunchArgument('namespace',default_value='mavros',description='Namespace for MAVROS node')

    # MAVROS node
    mavros_node = Node(
        package='mavros_telemetry_reader',
        executable='mavros_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
            {'target_system_id': LaunchConfiguration('tgt_system')},
            {'target_component_id': LaunchConfiguration('tgt_component')},
            {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
            # Load plugin lists from YAML
            os.path.join(get_package_share_directory('mavros_telemetry_reader'), 'param', 'apm_pluginlists.yaml'),
            # Load config from YAML
            os.path.join(get_package_share_directory('mavros_telemetry_reader'), 'param', 'apm_config.yaml'),
        ],
        # Conditionally respawn based on respawn_mavros argument
        respawn=LaunchConfiguration('respawn_mavros'),
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        # Launch arguments
        fcu_url,
        gcs_url,
        tgt_system,
        tgt_component,
        log_output,
        fcu_protocol,
        respawn_mavros,
        namespace,
        
        # MAVROS node
        mavros_node,
    ])