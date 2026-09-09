# 云端按量跑 Go2 仿真（Docker 一键环境）

本机（Intel Iris Xe 集显 / 内存紧张）跑重负载仿真太吃力，需要重仿真时换**按量计费云服务器**。
整套代码基于 **Ubuntu 26.04 + ROS 2 Lyrical + gz-sim 10**——多数云服务器出厂是 Ubuntu 22.04
（对应 ROS Humble / gz-sim 6，**跑不起这套代码**）。所以这里用 **Docker 镜像 = 纯工具链**：
镜像内只装环境，你的代码目录 bind-mount 进去，**换任何一台服务器都零重装、改代码不用重建镜像**。

> 计费提醒：按量服务器是**开着就扣钱**。用 `-A N` 让它在 N 分钟后自动关机（见下）；或用
> 抢占式/spot 实例；跑完把实例 **Stopped**（停止）才算停，光关窗口不算。

---

## 最快路径（一台全新 22.04/24.04/26.04 云服务器）

```bash
# 1) 一次性准备（装 docker；GPU 机加 --gpu）——需要 root/sudo，会要你输密码
git clone <你的仓库 URL> ros2_ws && cd ros2_ws
bash scripts/cloud_setup.sh            # 或 GPU 实例: bash scripts/cloud_setup.sh --gpu
#    装完要【重新登录一次】docker 组才生效（或执行 newgrp docker）

# 2) 首次构建镜像 + 工具链自检（构建要几分钟，只此一次）
bash scripts/cloud_run.sh -m selfcheck
#    预期看到 os = Ubuntu 26.04、ROS_DISTRO=lyrical、GL renderer = llvmpipe
#    （无 NVIDIA 时默认软件渲染，正常）

# 3) 自动压测（headless，约 1 分钟，无需 GPU）
bash scripts/cloud_run.sh -m bench -A 45
#    输出 RTF（实时因子）、/scan /imu 频率、内存水位、PASS/FAIL
#    RTF >= 0.85 这台机器就能流畅跑；-A 45 = 45 分钟后宿主机自动 poweroff

# 4) 想看画面（浏览器远程桌面）——需要在安全组放行 6080 端口
bash scripts/cloud_run.sh -m gui -A 60
#    然后浏览器打开:  http://<服务器公网IP>:6080/vnc.html
```

`cloud_run.sh` 所有参数：`bash scripts/cloud_run.sh -h`

| 参数 | 含义 | 例 |
|---|---|---|
| `-m` | 模式 `gui` / `headless` / `bench` / `selfcheck` | `-m bench` |
| `-d` | 跑几秒仿真后退出（headless 演示用） | `-d 30` |
| `-p` | 自定义楼层平面图（PGM），相对仓库根 | `-p scene/sample_floorplan.pgm` |
| `-r` | 平面图分辨率 m/像素 | `-r 0.30` |
| `-s` | 激光噪声 stddev | `-s 0.015` |
| `-A` | N 分钟后宿主机自动 **poweroff**（计费护栏） | `-A 45` |
| `-g` | 透传 NVIDIA GPU（`--gpus all`） | `-m gui -g` |
| `-n` | 自定义镜像名 | `-n go2-cloud:test` |
| `-D` | 后台运行（detach） | `-m gui -D` |

---

## 镜像构建机制（为什么换机零重装）

- 镜像 **`docker/Dockerfile`** = ubuntu:26.04 + ROS Lyrical（与你的宿主机 `dpkg` 同款包集）+ gz-sim + RViz + noVNC。**不 COPY 仓库**。
- 仓库以 `-v "$REPO":/workspace` 挂载进容器，入口脚本 **`docker/entrypoint.sh`** 先 `colcon build`
  （symlink 增量，秒级）再按 `MODE` 启动。
- 因此：**改代码/换场景 → 直接重跑 `cloud_run.sh`，无需重建镜像**。只有依赖变化才需要删镜像重建：
  `docker rmi go2-cloud:lyrical && bash scripts/cloud_run.sh -m selfcheck`

## 私有仓库如何 clone（不用把 token 发给我/贴聊天里）

推荐两种，任选：

- **SSH deploy key**（只读，最省事）：
  ```bash
  ssh-keygen -t ed25519 -f ~/.ssh/cloud_go2 -N ''
  cat ~/.ssh/cloud_go2.pub    # 复制到你的 Git 服务商 → 仓库 → Deploy keys
  ```
  然后在服务器 `~/.ssh/config` 加：
  ```
  Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/cloud_go2
  ```
- **HTTPS PAT**：自己建一个**只读** fine-grained token，在服务器终端里
  `git clone https://<你的用户名>:<token>@github.com/<org>/ros2_ws.git`
  （token 只出现在你自己的终端里，不会进本对话）。

## GPU / 显示说明（重要）

- **无 GPU（默认）**：ogre2 渲染走 `llvmpipe` 软件渲染，任何机器都能出图，CPU 占用高些。10 Hz 激光 + 100 Hz IMU + 相机在 4~8 vCPU 一般没问题，压测以 `-m bench` 的 RTF 为准。
- **NVIDIA GPU**：`setup.sh --gpu` + `cloud_run.sh -g`。GPU 主要加速**无窗口传感器渲染 / 物理**；窗口化 GLX 加速受容器限制，浏览器里看画面仍可能偏软渲染。GPU 机开 spot 抢占便宜很多。
- **无头（`headless`/`bench`）仍需 Xvfb**：容器内 `Xvfb :99` 保证传感器有 GL 上下文（入口脚本自动处理，不用你管）。

## bench 输出怎么读

```
lidar : 10 Hz | 340 finite rays / 360   # 场景够"堵"，雷达有回波
imu   : 100 Hz
RTF   : 0.98 (OK)   sim 5.9 s per 6.0 wall s   # 接近 1 = 实时
drive : odom x 0.000 -> 1.9 m at 0.5 m/s over 4 s (OK)   # 巡逻链路活着
VERDICT: PASS
```
`RTF < 0.85` 建议升 vCPU 或 `cloud_run.sh` 不带相机重跑；内存低于几百 MB 时换大内存规格。

## 常见坑

- **镜像缺失包**（构建期报 E: Unable to locate …）：ROS 源按官方 26.04 步骤配置；若 `packages.ros.org` 对 26.04 尚无 `lyrical` 发行版，把 Dockerfile 的 `FROM ubuntu:26.04` 换成 `FROM ros:lyrical-ros-base`（注释里已备好）即可，其余不变。
- **6080 打不开**：云厂商安全组/防火墙放行 TCP 6080（noVNC）。
- **容器立刻退出**：看 `MODE=selfcheck` 输出或 `docker logs <容器>`；九成是挂载路径不对（入口脚本找不到 `/workspace/docker/entrypoint.sh`）。
- **gz-sim 偶发启动僵死**：云主机重启再跑即可（本机同套流程也偶发）。

## 文件地图

```
docker/Dockerfile            镜像：ubuntu:26.04 + ROS Lyrical + gz + noVNC
docker/entrypoint.sh         容器入口：Xvfb + build + 按 MODE 启动
docker/README-CLOUD.md       本文档
scripts/cloud_setup.sh       宿主机一次性：装 docker / NVIDIA toolkit（sudo）
scripts/cloud_run.sh         宿主机每次：build+run、-A 自动关机护栏
scripts/cloud_bench.py       容器内自动压测（RTF/频率/内存/巡逻链路）
scripts/cloud_run.sh -h      参数速查
```
