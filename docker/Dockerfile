# go2-cloud -- pure toolchain image for the Go2 patrol sim.
#
# The image intentionally contains NO copy of the repo: code is bind-mounted at
# /workspace at runtime, so changing code never requires rebuilding the image.
# Rebuild only when the toolchain (ROS Lyrical / gz-sim / deps) changes.
#
# Build:  docker build -t go2-cloud -f docker/Dockerfile .
# Run:    scripts/cloud_run.sh  (wraps docker run with the bind mount)
#
# Base: Ubuntu 26.04 + ROS 2 Lyrical, exactly what this repo is written for.
# (If `ubuntu:26.04` is not on your registry yet, `ros:lyrical-ros-base` is an
# equivalent drop-in base -- everything below still applies.)

ARG BASE_IMAGE=ubuntu:26.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    ROS_DISTRO=lyrical \
    # Everything runs inside one container -> loop back is all we need.
    GZ_IP=127.0.0.1 ROS_LOCALHOST_ONLY=1 \
    DISPLAY=:99

# --- base tools -------------------------------------------------------------
RUN apt-get update && apt-get install -y \
        curl ca-certificates gnupg lsb-release \
        git python3-pip python3-venv build-essential \
    && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Lyrical apt repository (official recipe) -------------------------
# pin the OS codename used in the sources line (the ROS repo publishes per-Ubuntu)
RUN install -d /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /etc/apt/keyrings/ros.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/ros.gpg] http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# --- toolchain packages (names verified against this machine's install) ------
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ros-base \
        # gz-sim 10 + the gz-*-vendor chain are pulled in by this metapackage
        ros-${ROS_DISTRO}-gz-sim-vendor \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros2launch \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        # build the go2_driver (C++17, links against these ament targets)
        ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-trajectory-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        # Python side of go2_scan_noise + tools
        ros-${ROS_DISTRO}-rclpy \
        # colcon / ament build tooling
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# --- X / VNC / software-GL stack (cloud hosts have no display) ---------------
# llvmpipe (mesa) gives gz/rviz a working GL context on any CPU; noVNC exposes
# it in the browser. nvidia-container-toolkit's libglvnd is used instead when
# the container is started with `--gpus all`.
RUN apt-get update && apt-get install -y \
        xvfb fluxbox x11vnc novnc websockify \
        libgl1 libgl1-mesa-dri mesa-utils xauth \
    && rm -rf /var/lib/apt/lists/* \
    # passwordless, loopback-only VNC is fine: it is only reachable through the
    # noVNC port you explicitly publish
    && mkdir -p /root/.vnc

# Repo is mounted here by scripts/cloud_run.sh
WORKDIR /workspace

# The entry point is a file of the mounted repo so it always matches the code.
ENTRYPOINT ["/workspace/docker/entrypoint.sh"]
