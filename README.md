#  multi_robot_relay

## 📋 功能概述 | Overview

这个包作为**中转站（Relay Hub）**，解决多个机器人运行相同 ROS2 项目代码时的话题冲突问题。  
This package acts as a **Relay Hub** to resolve topic conflicts when multiple robots run the same ROS2 project code.

---

## 🌟 核心特性 | Core Features

### 🛰️ 1. 透明的话题中转 | Transparent Topic Relay
- 本地话题 → 全局话题（添加机器人命名空间）  
  Local topic → Global topic (adds robot namespace)  
- 全局话题 → 本地话题（移除命名空间）  
  Global topic → Local topic (removes namespace)  
- 无需修改原有代码  
  No need to modify existing code  

### 🧭 2. TF变换中转 | TF Relay
- 自动为 TF 框架添加机器人名称前缀  
  Automatically adds robot prefix to TF frames  
- 保持本地 TF 不变  
  Keeps local TF unchanged  
- 支持多机器人可视化  
  Supports multi-robot visualization  

### 🌍 3. 坐标系统统一（可选）| Coordinate System Unification (Optional)
- 处理不同机器人地图原点不一致问题  
  Handles map origin differences between robots  
- 支持位置偏移和旋转变换  
  Supports position offset and rotation transformation  
- 自动转换位置和目标点  
  Automatically transforms positions and goal poses  

### 🧩 4. 机器人注册管理 | Robot Registration Management
- 中央注册服务器  
  Central registry server  
- 心跳监控  
  Heartbeat monitoring  
- 机器人列表广播  
  Robot list broadcasting  

---

## 🏗️ 包结构 | Package Structure

```

multi_robot_relay/
├── config/
│   ├── relay_config.yaml          # 中转站配置 | Relay configuration
│   └── robots_example.yaml        # 机器人配置示例 | Example robot config
├── launch/
│   ├── robot_relay.launch.py      # 单机器人中转站启动 | Single robot relay
│   ├── central_registry.launch.py # 中央注册服务器启动 | Central registry
│   └── multi_robot_system.launch.py # 多机器人系统启动 | Multi-robot system
└── multi_robot_relay/
├── topic_relay_node.py         # 话题中转节点 | Topic relay node
├── tf_relay_node.py            # TF中转节点 | TF relay node
├── coordinate_transformer_node.py # 坐标转换节点 | Coordinate transformer
└── robot_registry_node.py      # 机器人注册节点 | Robot registry node

```

---

## 🚀 快速开始 | Quick Start

### 🧱 1. 编译包 | Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select multi_robot_relay
source install/setup.bash
```

### 🤖 2. 单个机器人使用 | Single Robot Usage

在每个机器人上运行：
Run on each robot:

```bash
ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=robot1
ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=robot2
ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=robot3
```

### 🧭 3. 启动原有功能 | Launch Existing Functions

中转站启动后，正常启动导航等功能（话题名保持不变）：
After launching the relay, start navigation as usual (topic names unchanged):

```bash
ros2 launch navigation navigation.launch.py
```

### 🕹️ 4. 外部控制 | External Control

从外部控制时使用带命名空间的话题：
Use namespaced topics for external control:

```bash
ros2 topic pub /robot1/goal_pose geometry_msgs/PoseStamped "..."
ros2 topic pub /robot2/goal_pose geometry_msgs/PoseStamped "..."
```

### 📍 5. 查看所有机器人位置 | View All Robot Positions

```bash
ros2 topic echo /robot1/odom
ros2 topic echo /robot2/odom
ros2 topic echo /robot3/odom
```

---

## ⚙️ 配置说明 | Configuration Guide

### 🗺️ 地图原点配置 | Map Origin Configuration

#### 情况1：所有机器人使用同一地图原点

**Case 1: All robots share the same map origin**

```bash
ros2 launch multi_robot_relay robot_relay.launch.py \
    robot_name:=robot1 \
    enable_coord_transform:=false
```

#### 情况2：机器人地图原点不同

**Case 2: Robots have different map origins**

```bash
ros2 launch multi_robot_relay robot_relay.launch.py \
    robot_name:=robot2 \
    enable_coord_transform:=true \
    map_offset_x:=1.5 \
    map_offset_y:=0.5 \
    map_offset_yaw:=0.0
```

---

### 🗨️ 话题配置 | Topic Configuration

编辑 `config/relay_config.yaml` 自定义中转话题：
Edit `config/relay_config.yaml` to customize relayed topics:

```yaml
uplink_topics:  # 本地 → 全局 | Local → Global
  - local_topic: "/odom"
    global_topic: "/{robot_name}/odom"
    msg_type: "nav_msgs/Odometry"
    qos: 10

downlink_topics:  # 全局 → 本地 | Global → Local
  - global_topic: "/{robot_name}/goal_pose"
    local_topic: "/goal_pose"
    msg_type: "geometry_msgs/PoseStamped"
    qos: 10
```

---

## 📊 话题映射关系 | Topic Mapping

### 上行中转（本地 → 全局）| Uplink (Local → Global)

| 本地话题                  | 全局话题                     | 说明     | Description       |
| --------------------- | ------------------------ | ------ | ----------------- |
| `/odom`               | `/robot1/odom`           | 里程计    | Odometry          |
| `/scan`               | `/robot1/scan`           | 激光雷达   | Laser scan        |
| `/amcl_pose`          | `/robot1/amcl_pose`      | 定位位姿   | Localization pose |
| `/controller/cmd_vel` | `/robot1/cmd_vel_status` | 速度指令状态 | Velocity status   |

### 下行中转（全局 → 本地）| Downlink (Global → Local)

| 全局话题                       | 本地话题                  | 说明   | Description       |
| -------------------------- | --------------------- | ---- | ----------------- |
| `/robot1/goal_pose`        | `/goal_pose`          | 导航目标 | Navigation goal   |
| `/robot1/initialpose`      | `/initialpose`        | 初始位姿 | Initial pose      |
| `/robot1/cmd_vel_override` | `/controller/cmd_vel` | 速度覆盖 | Velocity override |

---

## 🔧 高级使用 | Advanced Usage

### 启动中央注册服务器 | Launch Central Registry

```bash
ros2 launch multi_robot_relay central_registry.launch.py
```

### 查询在线机器人 | Query Online Robots

```bash
ros2 service call /multi_robot/get_robots std_srvs/Trigger
```

### 订阅机器人列表 | Subscribe to Robot List

```bash
ros2 topic echo /multi_robot/robot_list
```

### 自定义TF中转频率 | Customize TF Relay Rate

```bash
ros2 run multi_robot_relay tf_relay --ros-args -p robot_name:=robot1 -p publish_rate:=100.0
```

---

## 🎯 使用场景 | Use Cases

### 场景1：多机器人仓储物流 | Scenario 1: Multi-Robot Warehouse Logistics

```bash
ros2 launch multi_robot_relay robot_relay.launch.py robot_name:=robot1
ros2 launch navigation navigation.launch.py
```

### 场景2：协同巡检 | Scenario 2: Cooperative Inspection

```bash
ros2 topic echo /robot1/amcl_pose &
ros2 topic echo /robot2/amcl_pose &
python3 patrol_scheduler.py
```

---

## 🐛 故障排查 | Troubleshooting

### 问题1：话题没有中转 | Issue 1: Topic Not Relayed

检查 | Check:

```bash
ros2 node list
ros2 topic list
```

解决 | Fix:

* 检查机器人名称 | Check robot name
* 检查配置路径 | Check config path
* 查看日志 | Check logs

### 问题2：坐标系不统一 | Issue 2: Coordinate Misalignment

```bash
ros2 run tf2_ros tf2_echo map robot1/map
```

解决 | Fix:

* 确认偏移配置 | Verify offset config
* 启用坐标转换节点 | Enable transformer
* 校准相对位置 | Calibrate positions

### 问题3：TF变换错误 | Issue 3: TF Transformation Error

```bash
ros2 run tf2_tools view_frames.py
```

解决 | Fix:

* 检查 TF 中转节点 | Check TF relay node
* 检查配置 | Verify configuration
* 调整频率 | Adjust frequency

---

## 📝 开发说明 | Developer Notes

### 添加新的中转话题 | Add New Relay Topics

1. 编辑 `config/relay_config.yaml`
2. 添加到 `uplink_topics` 或 `downlink_topics`
3. 重启中转节点

### 自定义坐标转换 | Customize Coordinate Transform

```python
def transform_pose_local_to_global(self, x, y, yaw):
    # 自定义转换逻辑 | Custom transformation logic
    pass
