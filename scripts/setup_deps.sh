#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  git \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-rosdep

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep not found"
  exit 1
fi

sudo rosdep init 2>/dev/null || true
rosdep update

vcs import src < ros2.repos

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
