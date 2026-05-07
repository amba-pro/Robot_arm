#!/usr/bin/env bash
set -euo pipefail

echo "[ARM4] Updating apt index..."
sudo apt-get update

echo "[ARM4] Installing base dependencies..."
sudo apt-get install -y \
  python3 \
  python3-pip \
  python3-venv \
  python3-serial \
  python3-tk \
  git \
  curl \
  wget \
  lsb-release \
  gnupg2 \
  software-properties-common

echo "[ARM4] Installing Python packages..."
python3 -m pip install --upgrade pip --break-system-packages
python3 -m pip install pyserial customtkinter --break-system-packages

if ! command -v ros2 >/dev/null 2>&1; then
  UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
  ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-}"
  if [[ -z "${ROS_DISTRO_TARGET}" ]]; then
    case "${UBUNTU_CODENAME}" in
      jammy) ROS_DISTRO_TARGET="humble" ;;
      noble) ROS_DISTRO_TARGET="jazzy" ;;
      *)
        echo "[ARM4] Unsupported Ubuntu codename for auto ROS selection: ${UBUNTU_CODENAME}"
        echo "[ARM4] Set ROS_DISTRO_TARGET manually (e.g. jazzy or humble) and rerun."
        exit 1
        ;;
    esac
  fi
  echo "[ARM4] ROS 2 not found. Installing ROS 2 ${ROS_DISTRO_TARGET}..."
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  sudo apt-get update
  sudo apt-get install -y \
    "ros-${ROS_DISTRO_TARGET}-ros-base" \
    "ros-${ROS_DISTRO_TARGET}-rviz2" \
    "ros-${ROS_DISTRO_TARGET}-rqt" \
    "ros-${ROS_DISTRO_TARGET}-rqt-common-plugins" \
    python3-colcon-common-extensions
else
  echo "[ARM4] ROS 2 already installed, skipping ROS install."
  echo "[ARM4] Detected ROS_DISTRO=${ROS_DISTRO:-unknown} (existing installation will be used)."
fi

echo "[ARM4] Done. Verification:"
python3 --version
python3 -m pip show pyserial | sed -n '1,4p'
python3 -m pip show customtkinter | sed -n '1,4p' || true
if command -v ros2 >/dev/null 2>&1; then
  ros2 --version || true
fi
