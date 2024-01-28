#!/bin/bash

# Prerequisite: Ubuntu 22.04 environment
# Author: Daniel Asadi
# Description: One-time script for first time repo setup + ROS 2 Humble installation
# Usage: In dv24/ run 'bash scripts/utfr_init.sh'

install_ros2_humble(){
  echo "ROS 2 Humble is not installed. Installing ROS 2 Humble..."
  echo "Following installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
  
  #Locale setup
  echo "Setting up locale..."
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  #Setup sources
  echo "Setting up sources..."
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  #Install ROS 2 packages
  echo "Installing ROS 2 packages..."
  sudo apt update
  sudo apt upgrade
  sudo apt install ros-humble-desktop
  sudo apt install ros-dev-tools
}

# Check if ROS 2 Humble is installed
if ! [ -d "/opt/ros/humble" ]; then
  install_ros2_humble
  #get error code
  if [ $? -ne 0 ]; then
    echo "ROS 2 Humble installation failed. Please review error."
    echo "Installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
  else
    echo "ROS 2 Humble installation successful."
  fi
fi

# Check if ROS 2 Humble is sourced in ~/.bashrc
if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "Adding 'source /opt/ros/humble/setup.bash' to ~/.bashrc"
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
fi

# Init all submodules
echo "==========================================="
echo "=           Initalize submodules          ="
echo "==========================================="
git submodule update --init --recursive

# Update existing Packages:
echo "==========================================="
echo "=           Install Ubuntu deps           ="
echo "==========================================="
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install -y libcgal-dev libboost-all-dev ros-humble-rosbag2-storage-mcap clang-format

echo "==========================================="
echo "=            Install rosdeps              ="
echo "==========================================="
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "==========================================="
echo "=            Install pip deps             ="
echo "==========================================="
pip install xgboost scikit-learn imutils xacro

echo "==========================================="
echo "=               Config sim                ="
echo "==========================================="
DIR="$(pwd)/src/utfr_sim"
echo "export EUFS_MASTER=$DIR" >> ~/.bashrc
source ~/.bashrc

echo "==========================================="
echo "=     Config colcon to ignore drivers     ="
echo "==========================================="
YAML_CONTENT=$(cat <<EOF
{
  "build":
    {
      "packages-ignore":
        [
          "arena_camera_node",
          "bluespace_ai_xsens_mti_driver",
          "ouster_sensor_msgs",
          "ouster_ros",
          "launcher"
        ],
    },
  "test":
    {
      "packages-ignore":
        [
          "arena_camera_node",
          "bluespace_ai_xsens_mti_driver",
          "ouster_sensor_msgs",
          "ouster_ros",
          "launcher"
        ],
    },
}
EOF
)

COLCON_CONFIG_DIR="$HOME/.colcon"
mkdir $COLCON_CONFIG_DIR
echo "$YAML_CONTENT" > $COLCON_CONFIG_DIR/defaults.yaml

echo "FYI: to use nodes under drivers/, use the Jetson Orin to avoid additional setup"
echo "You are now ready to build via 'colcon build --<options>'"
echo "After a sucessful build, run 'source install/setup.bash'"
echo "Finally you can launch with 'ros2 launch <node name> <launch file name>'"
