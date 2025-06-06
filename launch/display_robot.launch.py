#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径
    pkg_share = get_package_share_directory('fishbot_description')
    
    # URDF xacro文件路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'fishbot', 'robot.urdf.xacro')
    
    # RViz配置文件路径
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_display.rviz')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 使用xacro处理URDF文件
    robot_description = Command(['xacro ', default_model_path])
    
    # 参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
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
    
    # joint_state_publisher_gui节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加所有动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    
    return ld