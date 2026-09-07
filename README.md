# ros2_ws

ROS 2 工作空间（Lyrical · Ubuntu 26.04）。

## 构建

```bash
source /opt/ros/lyrical/setup.bash
colcon build
source install/setup.bash
```

## 目录

- `src/` — 源码包（本仓库只跟踪此目录）
- `build/`、`install/`、`log/` — colcon 生成产物，已通过 `.gitignore` 忽略
