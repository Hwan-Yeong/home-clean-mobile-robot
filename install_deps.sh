#!/bin/bash
# ==============================================================================
# install_deps.sh
# Nav2 기반 청소 로봇 프로젝트 - 시스템 의존성 설치 스크립트
#
# 이 스크립트는 apt 패키지 매니저로 설치해야 하는 범용 유틸리티 패키지들을
# 한 번에 설치합니다. 소스 코드로 관리하는 핵심 패키지(Nav2, SLAM Toolbox,
# opennav_coverage 등)는 deps.repos + vcstool로 별도 관리합니다.
#
# Usage:
#   chmod +x install_deps.sh
#   ./install_deps.sh
# ==============================================================================

set -e

echo "============================================"
echo " [1/5] Updating apt package lists..."
echo "============================================"
sudo apt update

echo "============================================"
echo " [2/5] Installing ROS2 Humble packages..."
echo "============================================"
sudo apt install -y --no-install-recommends \
    gz-tools2- \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-map-server \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro

echo "============================================"
echo " [3/5] Installing vcstool..."
echo "============================================"
sudo apt install -y python3-vcstool python3-colcon-common-extensions

echo "============================================"
echo " [4/5] Installing Fields2Cover dependencies..."
echo "============================================"
sudo apt install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    doxygen \
    g++ \
    git \
    libeigen3-dev \
    libgdal-dev \
    libpython3-dev \
    python3 \
    python3-pip \
    python3-matplotlib \
    python3-tk \
    lcov \
    libgtest-dev \
    libtbb-dev \
    swig \
    libgeos-dev \
    gnuplot \
    libtinyxml2-dev \
    nlohmann-json3-dev

echo "============================================"
echo " [5/5] Installing Python dependencies..."
echo "============================================"
sudo apt install -y --no-install-recommends \
    python3-opencv \
    python3-numpy \
    python3-shapely \
    python3-transforms3d

echo ""
echo "============================================"
echo " All dependencies installed successfully!"
echo ""
echo " Next steps:"
echo "   1. cd ~/hyjoe_repositories/src/home-clean-mobile-robot"
echo "   2. vcs import . < deps.repos"
echo "   3. rosdep install --from-paths src --ignore-src -r -y"
echo "   4. colcon build --symlink-install"
echo "============================================"
