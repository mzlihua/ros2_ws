# ros2_ws

ROS 2 工作空间（Lyrical · Ubuntu 26.04），包含一个 Unitree Go2 四足机器人的
Gazebo Sim（gz-sim）仿真工程。

## 构建

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select go2_description go2_gazebo
source install/setup.bash
```

## 仿真运行（可驱动巡检版）

启动 gz-sim 仿真（go2 本体 + IMU/2D 激光/RGB+深度相机）、robot_state_publisher、
`go2_driver`（/cmd_vel→/odom+TF、腿部行走动画）与 ros_gz_bridge：

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# 无界面运行（若只在终端验证），或去掉 gui:=false 以打开 gz GUI + RViz
ros2 launch go2_gazebo go2_patrol.launch.py gui:=false

# 另一个终端里手动驾驶（或在启动命令加 demo:=15 自动巡游 15 秒）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

设计要点（零重力、纯运动学「可驱动巡检」）：

- world 为**零重力**，机器人本体由 gz-sim `VelocityControl` 系统按
  `/cmd_vel`（经 bridge 桥到 `/model/go2/cmd_vel`）滑行驱动，不会倾倒或触地；
- 腿部 12 个关节由 `JointTrajectoryController` 按
  `/go2/joint_trajectory` 动画成对角小跑步态，由 `go2_driver` 生成；
- 机器人自身不带碰撞体，纯运动学巡逻，传感器挂在机身上随滑行运动；
- 常用话题：`/scan` `/imu/data` `/camera/color/image_raw`
  `/camera/depth/image_raw` `/odom` `/joint_states`。

> 提示：若局域网组播不可用，请为 gz-sim 与本机 ROS 增加回环配置后运行：
> `export GZ_IP=127.0.0.1 ROS_LOCALHOST_ONLY=1`

## 目录

- `src/go2_description` — 官方 go2.urdf + 部件 CAD（dae）+ 话题/坐标系约定
- `src/go2_gazebo` — 巡检 world、URDF→SDF 转换器、`go2_driver`(C++)、
  bridge 配置、launch、RViz 布局
- `src/_deprecated` — 早期简化版 xacro 建模与旧 world（已弃用，仅存档）
- `build/`、`install/`、`log/` — colcon 生成产物，已通过 `.gitignore` 忽略
  （本仓库只跟踪 `src/` 目录与 README）
