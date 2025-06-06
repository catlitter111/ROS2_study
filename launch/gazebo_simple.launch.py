#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory('fishbot_description')
    
    # 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'fishbot', 'robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'custoom_room.world')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 使用xacro处理URDF文件
    robot_description = Command(['xacro ', default_model_path])
    
    # 参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 启动Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', default_world_path],
        output='screen'
    )
    
    # robot_state_publisher节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # 生成机器人到Gazebo中（添加延迟确保Gazebo完全启动）
    spawn_entity_cmd = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'fishbot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加所有动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    
    return ld
