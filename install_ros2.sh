#!/bin/bash

# Define variables
ROS2_DISTRO="jazzy"

# Check if the script is run with sudo privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root. Use 'sudo ./install_ros2.sh'."
   exit 1
fi

# Update package list
echo "Updating package list..."
apt update -y

echo "Set locale..."
apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

echo "Enable required respositories..."
apt install software-properties-common
add-apt-repository universe

echo "Adding ROS 2 GPG key..."
apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Install development tools..."
apt update -y && apt install ros-dev-tools

echo "Installing ROS 2 desktop package..."
apt update -y
apt upgrade -y
apt install ros-$ROS2_DISTRO-desktop
apt install python3-colcon-common-extensions python3-argcomplete ros-dev-tools

# Source the ROS 2 setup file
echo "Sourcing ROS 2 setup file..."
source /opt/ros/$ROS2_DISTRO/setup.bash

