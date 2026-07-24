"""
play.launch.py
---------------
Lanza el nodo player_node de trajectory_recorder.

Uso:
    ros2 launch trajectory_recorder play.launch.py
    ros2 launch trajectory_recorder play.launch.py filename:=vuelta1.json use_pid:=true kp_linear:=0.8
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value='trajectory.json',
        description='Nombre del archivo JSON a reproducir.',
    )
    input_dir_arg = DeclareLaunchArgument(
        'input_dir',
        default_value=os.path.expanduser('~/turtlebot3_ws/trajectories'),
        description='Carpeta donde buscar el archivo de trayectoria.',
    )
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='10.0',
        description='Frecuencia de publicación de cmd_vel en Hz.',
    )
    use_pid_arg = DeclareLaunchArgument(
        'use_pid',
        default_value='true',
        description='Activa la corrección de trayectoria por PID.',
    )

    kp_linear_arg = DeclareLaunchArgument('kp_linear', default_value='0.6')
    ki_linear_arg = DeclareLaunchArgument('ki_linear', default_value='0.0')
    kd_linear_arg = DeclareLaunchArgument('kd_linear', default_value='0.05')

    kp_angular_arg = DeclareLaunchArgument('kp_angular', default_value='1.2')
    ki_angular_arg = DeclareLaunchArgument('ki_angular', default_value='0.0')
    kd_angular_arg = DeclareLaunchArgument('kd_angular', default_value='0.1')

    max_lin_corr_arg = DeclareLaunchArgument(
        'max_linear_correction', default_value='0.1'
    )
    max_ang_corr_arg = DeclareLaunchArgument(
        'max_angular_correction', default_value='0.5'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom'
    )

    player_node = Node(
        package='trajectory_recorder',
        executable='player_node',
        name='trajectory_player_node',
        output='screen',
        parameters=[{
            'filename': LaunchConfiguration('filename'),
            'input_dir': LaunchConfiguration('input_dir'),
            'playback_rate': LaunchConfiguration('playback_rate'),
            'use_pid': LaunchConfiguration('use_pid'),
            'kp_linear': LaunchConfiguration('kp_linear'),
            'ki_linear': LaunchConfiguration('ki_linear'),
            'kd_linear': LaunchConfiguration('kd_linear'),
            'kp_angular': LaunchConfiguration('kp_angular'),
            'ki_angular': LaunchConfiguration('ki_angular'),
            'kd_angular': LaunchConfiguration('kd_angular'),
            'max_linear_correction': LaunchConfiguration('max_linear_correction'),
            'max_angular_correction': LaunchConfiguration('max_angular_correction'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
    )

    return LaunchDescription([
        filename_arg,
        input_dir_arg,
        playback_rate_arg,
        use_pid_arg,
        kp_linear_arg,
        ki_linear_arg,
        kd_linear_arg,
        kp_angular_arg,
        ki_angular_arg,
        kd_angular_arg,
        max_lin_corr_arg,
        max_ang_corr_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        player_node,
    ])
