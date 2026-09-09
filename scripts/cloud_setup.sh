#!/usr/bin/env bash
# cloud_setup.sh -- one-time host preparation for running the Go2 sim in Docker.
# Run ONCE per (re)built cloud server, as root or with sudo:
#   bash scripts/cloud_setup.sh            # docker only (CPU/headless/bench)
#   bash scripts/cloud_setup.sh --gpu      # + NVIDIA container toolkit
#
# After it finishes: log out/in (docker group), clone the repo, then use
#   bash scripts/cloud_run.sh  (see its --help).

set -euo pipefail

GPU=0
for a in "$@"; do
    case "$a" in
        --gpu) GPU=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown arg: $a"; exit 2 ;;
    esac
done

say() { printf '\033[1;34m[setup] %s\033[0m\n' "$*"; }
need_sudo() { if [ "$(id -u)" -eq 0 ]; then echo; else echo sudo; fi; }
SUDO=$(need_sudo)
[ -n "$SUDO" ] && { command -v sudo >/dev/null || { echo "run as root or install sudo"; exit 1; }; }

if ! command -v docker >/dev/null 2>&1; then
    say "installing docker.io ..."
    $SUDO apt-get update
    $SUDO apt-get install -y docker.io
fi
$SUDO systemctl enable --now docker
say "docker: $($SUDO docker --version)"

# allow current user to talk to the daemon without sudo
if [ -n "$SUDO" ]; then
    $SUDO usermod -aG docker "$USER" || true
    say "added '$USER' to group 'docker' -- log out and back in (or run newgrp docker)"
fi

if [ "$GPU" = "1" ]; then
    if ! command -v nvidia-smi >/dev/null 2>&1; then
        say "WARNING: no nvidia-smi on the HOST yet -- a GPU cloud image (NVIDIA driver) is expected"
    fi
    if ! dpkg -s nvidia-container-toolkit >/dev/null 2>&1; then
        say "installing nvidia-container-toolkit ..."
        $SUDO apt-get install -y curl gnupg
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
            | $SUDO gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
            | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
            | $SUDO tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
        $SUDO apt-get update
        $SUDO apt-get install -y nvidia-container-toolkit
    fi
    $SUDO nvidia-ctk runtime configure --runtime=docker
    $SUDO systemctl restart docker
    say "nvidia container runtime configured"
fi

cat <<'EOF'

Next steps
----------
  # clone this repo once, then you can switch servers freely:
  git clone <your-repo-url> ros2_ws && cd ros2_ws

  # quick sanity run (no GPU needed):
  bash scripts/cloud_run.sh -m selfcheck

  # automatic headless test:
  bash scripts/cloud_run.sh -m bench -A 45        # poweroff 45 min after exit

  # see the sim in a browser (needs the published port 6080):
  bash scripts/cloud_run.sh -m gui

Full docs: docker/README-CLOUD.md
EOF
