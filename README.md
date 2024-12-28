# Cloud Car 自动配送机器人

这是一个基于ROS2的自动配送机器人项目，实现了机器人的自主导航、远程监控和自动配送功能。

## 项目结构

```
src/
├── cloud_car/            # 机器人基础功能包
├── cloud_car_nav2/       # 导航相关功能包
└── cloud_car_delivery/   # 配送功能包
```

## 主要功能

- 🤖 自主导航：基于Nav2的自主导航系统
- 🎯 目标点导航：支持多点导航和路径规划
- 📹 远程监控：实时视频流传输
- 🌐 HTTP服务：提供RESTful API接口
- 📍 位置追踪：实时位置和状态更新
- 🔋 电量监控：电池状态实时监测
- 🚀 速度控制：实时速度信息反馈

## 技术栈

- ROS2 
- Python
- Flask
- Nav2
- OpenCV

## 安装依赖

1. 安装ROS2
```bash
# 请参考ROS2官方文档安装ROS2
```

2. 安装项目依赖
```bash
pip install flask opencv-python numpy
```

3. 编译工作空间
```bash
cd chapt5_ws
colcon build
source install/setup.bash
```

## 使用说明

1. 启动导航功能
```bash
ros2 launch cloud_car_nav2 navigation_launch.py
```

2. 启动配送节点
```bash
ros2 run cloud_car_delivery delivery_node
```

3. 启动HTTP服务器
```bash
ros2 run cloud_car_delivery http_server
```

## API接口

- `/robot/video_stream`: 获取实时视频流
- `/robot/state`: 获取机器人状态
- `/robot/arrived`: 到达目标点通知
- `/robot/pickup`: 取货状态查询
- `/robot/back`: 返回站点通知
- `/robot/velocity`: 速度信息获取

## 配送流程

1. 机器人空闲状态下接收配送任务
2. 根据目标点CSV文件进行导航
3. 到达目标点后等待取货
4. 取货完成后自动返回站点
5. 全程通过HTTP接口进行状态更新

## 维护说明

- 机器人支持维护模式
- 通过ROS2话题`/maintenance`发送维护指令
- 支持实时状态监控和远程调试

## 贡献指南

欢迎提交Issue和Pull Request来帮助改进项目。

## 许可证

MIT License 