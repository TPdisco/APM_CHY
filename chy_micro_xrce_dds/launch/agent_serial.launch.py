# launch/agent_serial_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 允许从命令行改串口和波特率
    dev_la  = DeclareLaunchArgument(
        'dev',  default_value='/dev/ttyUSB0',
        description='串口设备文件')
    baud_la = DeclareLaunchArgument(
        'baud', default_value='921600',
        description='串口波特率')

    # 直接 ExecuteProcess 拉起 MicroXRCEAgent
    agent = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'serial',
            '--dev',  LaunchConfiguration('dev'),
            '-b',     LaunchConfiguration('baud')
        ],
        output='screen',          # 日志打到当前终端
        emulate_tty=True,         # 颜色输出
        # 如果 Agent 异常退出，整个 launch 会重启它
        respawn=True,
        respawn_delay=2
    )

    return LaunchDescription([
        dev_la, 
        baud_la, 
        
        agent
        
        
        ])

