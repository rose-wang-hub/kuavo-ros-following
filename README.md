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

# 安装 LIVOX ROS 驱动
# 方法1：从源码安装（推荐）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
git checkout ros1
./build.sh ROS1
source ./devel/setup.bash

# 方法2：如果已有预编译包，直接source对应的setup.bash

# 安装 Python 依赖
pip install ultralytics opencv-python pyrealsense2 torch torchvision
```

3. 配置雷达：
```bash
# 确保雷达连接正常
# 检查雷达设备是否被识别
ls /dev/ttyUSB*  # 或其他串口设备

# 测试雷达连接（根据你的雷达型号调整参数）
roslaunch livox_ros_driver2 msg_MID360.launch
```

4. 构建工作空间：
```bash
cd kuavo-ros-following
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

**注意：需要同时运行两个launch文件（在不同的终端中）**

```bash
# 终端1：启动视觉跟踪系统
roslaunch kuavo_person_follow vision_tracking.launch

# 终端2：启动雷达跟随系统（包含LIDAR安全回避）
roslaunch kuavo_person_follow follow_robot.launch
```

**为什么需要两个launch文件？**

- `vision_tracking.launch`：负责视觉检测和跟踪，发布`/person_track/vision_state`
- `follow_robot.launch`：负责LIDAR驱动、障碍物检测和运动控制，订阅视觉状态进行跟随

两个系统协同工作：视觉提供目标位置，LIDAR确保安全跟随。

`follow_robot.launch` 会自动启动：
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

## 测试和验证

### 1. 测试雷达连接
```bash
# 启动雷达驱动
roslaunch livox_ros_driver2 msg_MID360.launch

# 检查话题
rostopic list | grep livox

# 查看雷达数据
rostopic echo /livox/lidar
```

### 2. 测试消息转换
```bash
# 启动转换器
rosrun livox_ros_driver2 livox_converter.py

# 检查PointCloud2话题
rostopic list | grep cloud

# 查看转换后的点云数据
rostopic echo /livox/cloud
```

### 3. 测试激光扫描转换
```bash
# 启动激光扫描转换
roslaunch pointcloud_to_laserscan sample_node.launch cloud_in:=/livox/cloud

# 检查激光扫描话题
rostopic list | grep scan

# 查看激光扫描数据
rostopic echo /scan
```

### 4. 测试视觉跟踪
```bash
# 启动视觉跟踪
roslaunch kuavo_person_follow vision_tracking.launch

# 检查视觉状态话题
rostopic list | grep person_track
```

### 5. 完整系统测试
```bash
# 终端1：视觉跟踪
roslaunch kuavo_person_follow vision_tracking.launch

# 终端2：跟随控制
roslaunch kuavo_person_follow follow_robot.launch

# 监控控制命令
rostopic echo /cmd_vel
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

1. **找不到 livox_ros_driver2 包**:
   ```bash
   # 确保已安装并source了livox驱动的工作空间
   source /path/to/livox_ws/devel/setup.bash
   ```

2. **雷达无法连接**:
   ```bash
   # 检查串口权限
   sudo chmod 666 /dev/ttyUSB*
   
   # 检查雷达IP和端口配置
   # 修改 livox 配置文件中的雷达参数
   ```

3. **LIDAR 数据不显示**:
   ```bash
   # 检查雷达是否正确连接
   rostopic list | grep livox
   
   # 查看雷达驱动日志
   rosnode info /livox_lidar_publisher2
   ```

4. **消息转换失败**:
   ```bash
   # 检查 livox_converter.py 是否可执行
   ls -la /path/to/livox_ws/src/livox_ros_driver2/scripts/livox_converter.py
   
   # 手动测试转换器
   rosrun livox_ros_driver2 livox_converter.py
   ```

5. **激光扫描无数据**:
   ```bash
   # 检查点云到激光转换的参数
   rostopic echo /livox/cloud | head -20
   
   # 调整转换参数（高度范围等）
   ```

6. **跟踪框不显示**:
   ```bash
   # 检查摄像头连接
   ls /dev/video*
   
   # 测试 YOLO 模型
   python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
   ```

7. **机器人不跟随**:
   ```bash
   # 检查视觉状态话题
   rostopic echo /person_track/vision_state
   
   # 检查控制命令输出
   rostopic echo /cmd_vel
   
   # 确认 PID 参数设置
   ```

### 日志查看

```bash
# 查看所有ROS日志
rostopic echo /rosout

# 查看特定节点日志
rosnode info /follow_controller

# 查看LIDAR数据
rostopic echo /scan | head -20

# 查看控制命令
rostopic echo /cmd_vel
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