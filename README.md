# ros2_ws

ROS 2 工作空间（Lyrical · Ubuntu 26.04），包含一个 Unitree Go2 四足机器人的
Gazebo Sim（gz-sim）仿真工程。场景为**室内仓库巡检**版：机器人本体为纯运动学
滑行驱动（零重力，不会倒/不会触地），腿部 12 关节按对角小跑步态动画；室内由
静态墙体/货架/货箱构成，激光、RGB+深度相机、IMU 可回真实室内量测。

## 构建

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select go2_description go2_gazebo
source install/setup.bash
```

## 仿真运行（可驱动巡检版）

启动 gz-sim（室内场景 + go2 本体 + IMU/2D 激光/RGB+深度相机）、
`robot_state_publisher`、`go2_driver`（/cmd_vel→/odom+TF、腿部行走动画）与
ros_gz_bridge（外加 `go2_scan_noise` 激光噪声重发）：

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# 无界面运行（若只在终端验证），或去掉 gui:=false 以打开 gz GUI + RViz
ros2 launch go2_gazebo go2_patrol.launch.py gui:=false

# 另一个终端里手动驾驶（或在启动命令加 demo:=15 自动巡游 15 秒）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 换成你自己的平面图（占用栅格图）

把一张 PGM 占用栅格图（**黑=墙**，ROS map_server 惯例；若你的图黑白相反请加
`--invert`）直接喂给 launch，脚本会生成对应室内墙体并替换默认仓库：

```bash
ros2 launch go2_gazebo go2_patrol.launch.py \
    scene_map:=install/go2_gazebo/share/go2_gazebo/scene/sample_floorplan.pgm \
    scene_res:=0.3
# 分辨率按你地图的米/像素设；地图中心默认放在世界原点（go2 出生点），
# 请保证中心像素在空地上
```

通用导入工具也可离线使用（`--apply-to` 覆盖世界副本，`sample` 重新生成默认仓库）：

```bash
python3 src/go2_gazebo/scripts/go2_scene.py sample --world worlds/go2_patrol.sdf
python3 src/go2_gazebo/scripts/go2_scene.py import --pgm my_map.pgm --res 0.05 \
    --invert --apply-to /tmp/out.sdf
```

## 设计要点

- **world 为零重力**，机器人本体由 gz-sim `VelocityControl` 系统按 `/cmd_vel`
  （经 bridge 桥到 `/model/go2/cmd_vel`）滑行驱动，腿部动画叠加其上，不会倾倒触地；
- 静态室内场景（墙体/货架/货箱/隔间+门洞）由 `go2_scene.py` 生成：默认仓库世界
  已含布局，`scene_map:=` 可从任意占用栅格重建——场景只生成在
  `<!-- SCENE_BEGIN … SCENE_END -->` 区段内，不动插件/光照/地面；
- 传感器真实化（本机流畅档）：IMU 在 SDF 内加 per-axis 高斯噪声；**gz GPU 激光不
  支持原生噪声**，故由 `go2_scan_noise` 在 ROS 侧为 `/scan` 加高斯噪声
  （`/scan_raw` 保留干净回波，`scan_noise:=0.0` 可关闭）；相机无干净噪声通道，
  真实感靠光照/材质——单方向光 + 两盏无阴影补光，默认关阴影省集显算力；
- 机器人自身不带碰撞体，纯运动学巡逻，传感器挂在机身上随滑行运动；
- 常用话题：`/scan`（带噪）`/scan_raw`（干净）`/imu/data`
  `/camera/color/image_raw` `/camera/depth/image_raw` `/odom` `/joint_states`。

> 提示：若局域网组播不可用，请为 gz-sim 与本机 ROS 增加回环配置后运行：
> `export GZ_IP=127.0.0.1 ROS_LOCALHOST_ONLY=1`

### 渲染/性能（本机流畅优先）

当前硬件为集显 + 内存紧张，场景刻意保持轻量：简单 box 几何、静态体合并在少量
static model、无阴影、无大纹理、相机 640×480、激光 360@10Hz。若想截图开阴影：
把 `worlds/go2_patrol.sdf` 中 `sun` 的 `cast_shadows` 临时改 `true`。

## 后续（未做，计划放云服务器）

- **C：真实物理落地行走**（重力 + 脚底碰撞 + 步态平衡控制）：改动大、需长时间
  调参，且本机集显/内存跑不动 dartsim 实时接触求解，属独立工程，将放云服务器跑。

## 目录

- `src/go2_description` — 官方 go2.urdf + 部件 CAD（dae）+ 话题/坐标系约定
- `src/go2_gazebo` — 室内场景与生成/导入脚本（`go2_scene.py`）、URDF→SDF 转换器、
  `go2_driver`(C++)、`go2_scan_noise`(激光噪声)、bridge 配置、launch、RViz 布局
- `src/_deprecated` — 早期简化版 xacro 建模与旧 world（已弃用，仅存档）
- `build/`、`install/`、`log/` — colcon 生成产物，已通过 `.gitignore` 忽略
  （本仓库只跟踪 `src/` 目录与 README）
