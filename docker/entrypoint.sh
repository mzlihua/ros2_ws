#!/usr/bin/env bash
# Container entry point for the Go2 cloud sim.  The repo is bind-mounted at
# /workspace; this script builds the two packages and runs one of:
#   MODE=gui        gz GUI + RViz on Xvfb, streamed to the browser via noVNC
#   MODE=headless   same sim without windows (sensors still render on Xvfb)
#   MODE=bench      headless sim + scripts/cloud_bench.py, prints PASS/FAIL
#   MODE=selfcheck  print toolchain/GL versions and exit
#
# Environment (optional): DEMO (patrol seconds), SCENE_MAP / SCENE_RES
# (own floor plan), SCAN_NOISE (laser noise stddev), SKIP_BUILD=1.

set -uo pipefail

MODE="${MODE:-headless}"
DEMO="${DEMO:-0.0}"
SCENE_MAP="${SCENE_MAP:-}"
SCENE_RES="${SCENE_RES:-0.30}"
SCAN_NOISE="${SCAN_NOISE:-0.015}"
SKIP_BUILD="${SKIP_BUILD:-0}"
WS=/workspace

log() { printf '\033[1;34m[go2]%s\033[0m\n' "$*"; }
die() { printf '\033[1;31m[go2] %s\033[0m\n' "$*" >&2; exit 1; }

# --- ROS environment ---------------------------------------------------------
[ -f /opt/ros/lyrical/setup.bash ] || die "no /opt/ros/lyrical/setup.bash in image"
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
[ -x /workspace/docker/entrypoint.sh ] || die "/workspace looks unmounted (no docker/entrypoint.sh)"
cd "$WS"

# --- Xvfb for ALL modes (ogre2 sensors need a GL context even headless) ------
pids=()
cleanup() {
    for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

start_xvfb() {
    if [ ! -S /tmp/.X11-unix/X99 ]; then
        Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
        pids+=($!)
        sleep 1
    fi
    export DISPLAY=:99
}

start_vnc() {
    fluxbox >/tmp/fluxbox.log 2>&1 &
    pids+=($!)
    x11vnc -display :99 -forever -shared -nopw -quiet -rfbport 5900 \
        >/tmp/x11vnc.log 2>&1 &
    pids+=($!)
    websockify --web /usr/share/novnc 0.0.0.0:6080 localhost:5900 \
        >/tmp/websockify.log 2>&1 &
    pids+=($!)
    sleep 1
    log "GUI: open http://<this-host>:6080/vnc.html in your browser"
}

# --- build (incremental, few seconds after the first) -------------------------
build_ws() {
    if [ "$SKIP_BUILD" = "1" ] && [ -f install/setup.bash ]; then
        log "SKIP_BUILD=1 -> using existing install/"
    else
        log "colcon build (go2_description, go2_gazebo)"
        colcon build --symlink-install \
            --packages-select go2_description go2_gazebo || die "colcon build failed"
    fi
    # shellcheck disable=SC1091
    source install/setup.bash
}

launch_args() {
    # "$@" are extra launch substitutions; first decides the GUI.
    local args=("$1" "demo:=$DEMO")
    [ -n "$SCENE_MAP" ] && args+=("scene_map:=$SCENE_MAP" "scene_res:=$SCENE_RES")
    args+=("${@:2}")
    printf '%s\n' "${args[@]}"
}

# ============================================================= selfcheck ====
if [ "$MODE" = "selfcheck" ]; then
    log "toolchain self check"
    echo "--- os ---";      . /etc/os-release && echo "$PRETTY_NAME ($UBUNTU_CODENAME)"
    echo "--- ros ---";     echo "ROS_DISTRO=$ROS_DISTRO  $(ls -d /opt/ros/lyrical >/dev/null && echo /opt/ros/lyrical OK)"
    echo "--- gz-sim pkg ---"
    dpkg -l "ros-${ROS_DISTRO}-gz-sim-vendor" 2>/dev/null | awk 'NR>=6{print $2, $3}'
    echo "--- colcon ---";  colcon version-check >/dev/null 2>&1 || true; colcon --help >/dev/null 2>&1 && echo "colcon OK"
    start_xvfb
    echo "--- GL renderer ---"
    if command -v glxinfo >/dev/null; then glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL version" || echo "(glxinfo: no GL context yet)"; else echo "(glxinfo not installed)"; fi
    echo "--- vnc tools ---"
    for b in Xvfb fluxbox x11vnc websockify; do command -v "$b" >/dev/null && echo "$b OK" || echo "$b MISSING"; done
    echo "selfcheck done"
    exit 0
fi

# ========================================================= build + launch ====
start_xvfb
if [ "$MODE" = "gui" ]; then
    start_vnc
    GUI_FLAG=gui:=true
else
    GUI_FLAG=gui:=false
fi

build_ws

# assemble launch arguments (ros2 launch passes key:=value pairs)
readarray -t ARGS <<< "$(launch_args "$GUI_FLAG")"

if [ "$MODE" = "gui" ] || [ "$MODE" = "headless" ]; then
    log "running go2_patrol.launch.py ${ARGS[*]}"
    # foreground so container lifetime == sim lifetime
    ros2 launch go2_gazebo go2_patrol.launch.py "${ARGS[@]}"
    rc=$?
    log "sim exited (rc=$rc)"
    exit $rc
fi

if [ "$MODE" = "bench" ]; then
    log "bench: starting headless sim, then cloud_bench.py"
    ros2 launch go2_gazebo go2_patrol.launch.py "${ARGS[@]}" \
        >/tmp/sim.log 2>&1 &
    SIM=$!
    pids+=($SIM)
    python3 "$WS/scripts/cloud_bench.py"
    rc=$?
    kill -INT "$SIM" 2>/dev/null || true
    sleep 2
    log "bench result rc=$rc"
    exit $rc
fi

die "unknown MODE='$MODE' (gui|headless|bench|selfcheck)"
