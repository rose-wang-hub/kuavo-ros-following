# ROS 人跟随系统 (ROS Person Following System)

这是一个完整的 ROS 人跟随系统，集成了视觉跟踪、PID 控制和 LIDAR 障碍物回避功能。

## 功能特性

- **视觉跟踪**: 使用 YOLOv8 进行人检测，结合 KCF 跟踪器实现稳定跟踪
- **PID 控制**: 实现平滑的跟随控制，包括距离和角度控制
- **LIDAR 障碍物回避**: 使用 LIVOX LIDAR 检测前方障碍物，确保安全跟随
- **集成启动**: 一键启动所有组件

## 系统架构

```
LIVOX LIDAR → CustomMsg → livox_converter.py → PointCloud2 → pointcloud_to_laserscan → LaserScan → follow_controller_node.py
                                                                                                      ↓
视觉跟踪 (YOLOv8 + KCF) → PersonState → follow_controller_node.py → /cmd_vel
```

## 硬件要求

- ROS Noetic
- LIVOX LIDAR (支持 CustomMsg 输出)
- Intel RealSense 摄像头
- Ubuntu 20.04

## 软件依赖

- Python 3.10
- OpenCV 4.12.0
- Ultralytics YOLOv8
- PyTorch
- Pyrealsense2
- LIVOX ROS Driver 2
- pointcloud_to_laserscan

## 安装步骤

1. 克隆仓库：
```bash
git clone https://github.com/rose-wang-hub/kuavo-ros-following.git
cd kuavo-ros-following
```

2. 安装依赖：
```bash
# 安装 ROS 包
sudo apt-get install ros-noetic-pointcloud-to-laserscan

# 安装 Python 依赖
pip install ultralytics opencv-python pyrealsense2 torch torchvision
```

3. 构建工作空间：
```bash
catkin_make
source devel/setup.bash
```

## 使用方法

### 环境设置

在使用系统之前，需要确保ROS环境正确配置，包括livox驱动：

```bash
# 设置ROS环境
source /opt/ros/noetic/setup.bash

# 设置livox驱动环境（假设安装在BIGAI目录）
source /home/kuavo/BIGAI/livox_driver/devel/setup.bash

# 设置跟随系统环境
cd /home/kuavo/kuavo-ros-following
source devel/setup.bash
```

### 一键启动完整系统

```bash
# 启动视觉跟踪（在第一个终端）
roslaunch kuavo_person_follow vision_tracking.launch

# 启动雷达跟随系统（在第二个终端）
roslaunch kuavo_person_follow follow_robot.launch
```

这个启动文件会自动启动：
- LIVOX LIDAR 驱动
- 消息转换器 (CustomMsg → PointCloud2)
- 点云到激光扫描转换
- 跟随控制器（包含雷达安全回避）

### 单独启动组件

1. 启动视觉跟踪：
```bash
roslaunch kuavo_person_follow vision_tracking.launch
```

2. 启动雷达跟随系统：
```bash
roslaunch kuavo_person_follow follow_robot.launch
```

## 消息接口

### 订阅话题

- `/livox/lidar` (livox_ros_driver2/CustomMsg): LIDAR 原始数据
- `/scan` (sensor_msgs/LaserScan): 激光扫描数据用于障碍物检测
- `/person_state` (kuavo_person_follow/PersonState): 跟踪目标状态

### 发布话题

- `/cmd_vel` (geometry_msgs/Twist): 速度控制命令

## 参数调优

### PID 控制参数

在 `follow_controller_node.py` 中调整：

```python
# 距离控制 PID
self.distance_pid = PIDController(kp=0.5, ki=0.1, kd=0.1, setpoint=1.5)

# 角度控制 PID
self.angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.2, setpoint=0.0)
```

### 安全参数

```python
# 最小前方距离 (米)
self.min_front_dist = 0.5

# 最大跟随距离 (米)
self.max_follow_dist = 3.0
```

## 故障排除

### 常见问题

1. **LIDAR 数据不显示**: 检查 LIVOX 驱动是否正确启动，确认网络连接
2. **跟踪框不显示**: 确保 RealSense 摄像头正常工作，检查 YOLO 模型加载
3. **机器人不跟随**: 检查 `/cmd_vel` 话题是否有数据输出，确认 PID 参数设置

### 日志查看

```bash
# 查看控制器日志
rostopic echo /rosout

# 查看 LIDAR 数据
rostopic echo /scan
```

## 开发说明

### 添加新功能

1. 在 `src/kuavo_person_follow/src/` 中添加新的 ROS 节点
2. 更新 `CMakeLists.txt` 和 `package.xml`
3. 修改启动文件以包含新节点

### 自定义消息

PersonState 消息定义在 `msg/PersonState.msg` 中，可以根据需要扩展。

## 许可证

本项目采用 MIT 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request 来改进这个系统。