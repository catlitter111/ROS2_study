# Fishbot Description Package

这个包包含了fishbot机器人的URDF描述文件和相关的launch文件。

## 文件结构

```
fishbot_description/
├── launch/
│   ├── display_robot.launch.py      # 在RViz中显示机器人模型
│   ├── gazebo_robot.launch.py       # 在Gazebo中启动机器人仿真（完整版）
│   └── gazebo_simple.launch.py      # 在Gazebo中启动机器人仿真（简化版）
├── urdf/
│   └── fishbot/
│       ├── robot.urdf.xacro         # 主机器人描述文件
│       ├── base.urdf.xacro          # 机器人底盘描述
│       └── sensor/                  # 传感器描述文件
├── worlds/
│   └── custoom_room.world           # Gazebo世界文件
├── rviz/
│   └── robot_display.rviz           # RViz配置文件
└── README.md
```

## 使用方法

### 1. 在RViz中显示机器人模型

```bash
ros2 launch fishbot_description display_robot.launch.py
```

这将启动：
- robot_state_publisher节点
- joint_state_publisher_gui节点
- RViz2

### 2. 在Gazebo中仿真机器人（推荐）

```bash
ros2 launch fishbot_description gazebo_robot.launch.py
```

这将启动：
- Gazebo仿真器
- 加载自定义房间世界
- 生成机器人模型到Gazebo中
- robot_state_publisher节点

### 3. 在Gazebo中仿真机器人（简化版）

```bash
ros2 launch fishbot_description gazebo_simple.launch.py
```

这是一个简化版本，直接使用ExecuteProcess启动Gazebo。

## 可用参数

### gazebo_robot.launch.py 参数

- `use_sim_time`: 是否使用仿真时间（默认：true）
- `world`: 世界文件路径（默认：custoom_room.world）
- `gui`: 是否显示Gazebo GUI（默认：true）
- `headless`: 是否无头运行（默认：false）

示例：
```bash
ros2 launch fishbot_description gazebo_robot.launch.py gui:=false
```

### display_robot.launch.py 参数

- `use_sim_time`: 是否使用仿真时间（默认：false）

## 机器人特性

- 四轮差速驱动机器人
- 配备激光雷达传感器
- 配备摄像头传感器
- 配备IMU传感器
- 支持Gazebo物理仿真
- 包含差速驱动控制器插件

## 控制机器人

在Gazebo仿真中，可以通过发布cmd_vel话题来控制机器人：

```bash
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"
```

## 依赖项

确保已安装以下ROS2包：
- robot_state_publisher
- joint_state_publisher_gui
- rviz2
- xacro
- gazebo_ros_pkgs

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select fishbot_description
source install/setup.bash
```
