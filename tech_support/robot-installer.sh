#!/bin/bash
#
# Installer script for ros2 and this robot on Ubuntu 22.04
#
##########################################################

case "$1" in
  "sudo")
    echo "Adding you to sudoers so you won't have to always type in sudo password..."
    sudo echo "$UID ALL=(ALL:ALL)   NOPASSWD:ALL" >> /etc/sudoers.d/$UID-user
    echo "Now log out and log back in for settings to take effect."
    ;;
  "gazebo")
    # Gazebo is seperate because it doesn't run on RPi4, this is more laptop/desktop use
    echo "Installing Gazebo related stuffs..."
    sudo apt install -y gazebo ros-humble-gazebo ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control
    ;;
  "ros2")
    echo "Installing ROS2 Humble, and friends..."
    sudo apt update && sudo apt upgrade -y # Do a freash update before we begin
    sudo apt remove brltty -y # Braille TTY library conflicts with arduinos
    sudo snap install code
    sudo snap install arduino
    # Now install ROS related stuffs
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    sudo apt install software-properties-common joystick evtest -y
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl minicom libserial-dev terminator -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade
    sudo apt install ros-humble-desktop ros-dev-tools -y
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source .bashrc
    sudo apt install ros-humble-nav2 ros-humble-nav2 ros-humble-nav2-rviz-plugins ros-humble-rplidar-ros ros-humble-ament*-y
    sudo apt install joint-state-publisher-gui ros-humble-ros2-control ros-humble-usb-cam ros-humble-xacro -y
    sudo apt install ros-humble-twist-mux ros-humble-ros2-controllers ros-humble-nav2-amcl ros-humble-slam-toolbox -y
    cd ~/
    mkdir -p dev_ws/src
    cd dev_ws && colcon build --symlink-install
    echo "ROS2 Installed, Enjoy the robot build!"
    echo "A dev workspace has been created at ~/dev_ws"
    ;;
  "robot")
    echo "Installing raptor_robot_v2 latest software..."
    mkdir -p ~/dev_ws/src # in case it's not there
    cd ~/dev_ws/src
    rm -rf raptor_robot_v2 # I know it's tradgic, but trust me it gets better
    git clone --branch Niagra https://github.com/jazzyengineermc/raptor_robot_v2.git # See, I told you so...
    cd ~/dev_ws && colcon build --symlink-install
    echo "Your robot awaits you"
    echo "source install/setup.bash"
    ;;
  *)
    echo "Usage: $0 {sudo|ros2|gazebo|robot}"
    exit 1
    ;;
esac
