#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory('fishbot_description')
    
    # 文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'fishbot', 'robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'worlds', 'custoom_room.world')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world_path)
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    
    # 使用xacro处理URDF文件
    robot_description = Command(['xacro ', default_model_path])
    
    # 参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to execute gzclient'
    )
    
    # 启动Gazebo服务器
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false'
        }.items()
    )
    
    # 启动Gazebo客户端
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ),
        condition=IfCondition(gui)
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
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_headless_cmd)
    
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    
    return ld
