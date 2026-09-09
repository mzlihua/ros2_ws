#!/usr/bin/env bash
# cloud_run.sh -- build (once) and run the Go2 sim container on THIS host.
#
# The image is a pure toolchain (ROS Lyrical + gz-sim); your repo is bind-mounted
# in, so code edits never need an image rebuild.  Anything cloud-vendor-specific
# is out of scope: this runs wherever docker exists.
#
# Examples
#   bash scripts/cloud_run.sh -m selfcheck            # sanity: versions + GL
#   bash scripts/cloud_run.sh -m bench                # automatic headless test
#   bash scripts/cloud_run.sh -m bench -A 45          # ... then poweroff 45 min after
#   bash scripts/cloud_run.sh -m gui                  # browser UI at :6080
#   bash scripts/cloud_run.sh -m headless -d 20       # run sim 20 s demo, headless
#   bash scripts/cloud_run.sh -m gui -g               # + NVIDIA GPU passthrough
#   bash scripts/cloud_run.sh -p scene/sample_floorplan.pgm -m bench   # own floor plan

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMG="${IMG:-go2-cloud:lyrical}"
MODE=headless
DEMO=0.0
SCENE_MAP=
SCENE_RES=0.30
SCAN_NOISE=0.015
GPU=0
DETACH=0
AUTO_OFF=
SKIP_BUILD=0

usage() { sed -n '2,/^$/p' "$0"; exit "${1:-0}"; }

while getopts ":m:d:p:r:s:A:n:gDh" o; do
    case "$o" in
        m) MODE=$OPTARG ;;
        d) DEMO=$OPTARG ;;
        p) SCENE_MAP=$OPTARG ;;
        r) SCENE_RES=$OPTARG ;;
        s) SCAN_NOISE=$OPTARG ;;
        A) AUTO_OFF=$OPTARG ;;
        n) IMG=$OPTARG ;;
        g) GPU=1 ;;
        D) DETACH=1 ;;
        h) usage 0 ;;
        *) usage 2 ;;
    esac
done
shift $((OPTIND - 1))
case "$MODE" in gui|headless|bench|selfcheck) ;; *) echo "bad mode: $MODE"; usage 2 ;; esac

command -v docker >/dev/null 2>&1 || { echo "docker missing -- run scripts/cloud_setup.sh first"; exit 1; }

# --- resolve scene map to an absolute host path (must live in the repo) ------
if [ -n "$SCENE_MAP" ]; then
    if [ ! -f "$REPO/$SCENE_MAP" ]; then
        echo "scene map not found: $REPO/$SCENE_MAP"; exit 1
    fi
    SCENE_MAP="/workspace/$SCENE_MAP"   # container sees the repo at /workspace
fi

# --- build image on first use -------------------------------------------------
if ! docker image inspect "$IMG" >/dev/null 2>&1; then
    echo ">> building toolchain image $IMG (first run only, several minutes)..."
    docker build -t "$IMG" -f "$REPO/docker/Dockerfile" "$REPO"
fi

# --- optional host auto-poweroff guardrail (metered billing!) ------------------
if [ -n "$AUTO_OFF" ]; then
    case "$AUTO_OFF" in *[!0-9]*) echo "auto-off minutes must be an integer"; exit 2 ;; esac
    ( sleep "${AUTO_OFF}m" && { [ "$(id -u)" = 0 ] && poweroff || sudo -n poweroff || poweroff; } ) &
    disown || true
    echo ">> scheduled host poweroff in ${AUTO_OFF} min (pid $!)"
fi

# --- docker run ---------------------------------------------------------------
IT=(); [ -t 0 ] && [ -t 1 ] && IT=(-it)
GPUS=(); [ "$GPU" = "1" ] && GPUS=(--gpus all)
DET=(); [ "$DETACH" = "1" ] && DET=(-d)

set -x
docker run "${IT[@]}" "${DET[@]}" --rm \
    --shm-size=1g \
    -p 6080:6080 \
    -v "$REPO":/workspace \
    "${GPUS[@]}" \
    -e MODE="$MODE" -e DEMO="$DEMO" \
    -e SCENE_MAP="$SCENE_MAP" -e SCENE_RES="$SCENE_RES" \
    -e SCAN_NOISE="$SCAN_NOISE" -e SKIP_BUILD="$SKIP_BUILD" \
    "$IMG"
set +x

if [ "$MODE" = "gui" ]; then
    echo ">> if it is still running: open  http://<this-host-public-ip>:6080/vnc.html"
fi
