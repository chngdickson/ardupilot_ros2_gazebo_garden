## Install ROS2 Humble Binary + Gazebo Garden from source in Ubuntu 22.04

RMB to setup
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/

ROS From Binary
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 

Gazebo from Source
https://gazebosim.org/docs/garden/install_ubuntu_src

Ros_Gz Linking Gazebo and ROS
https://github.com/gazebosim/ros_gz/tree/humble#from-source

## 1. Install ROS2 Humble
Set Locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale 
```

This code enables Ubuntu Universe Repos
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add Ros2 GPG Key
```
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install packages
```
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop
```

Go to bash.rc file in your ubuntu Home, add this line
```
source /opt/ros/humble/setup.bash
```

Test examples Open New terminal
Talker Terminal 1
``` 
ros2 run demo_nodes_cpp talker
```
Listener Terminal 2
```
ros2 run demo_nodes_py listener
```

## 2. Install Gazebo Garden from source 
link
```https://gazebosim.org/docs/garden/install_ubuntu_src```
Export GZ Version, used for Gazebo version the ROS Would like to compile against
Put these in your bash.rc file in Home
```
source /opt/ros/humble/setup.bash
export GZ_VERSION=garden
```

install necessary tools
```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```

install gazebo Garden
```
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

## 3. Install ros_gz to link gazebo and ros
```
cd
mkdir -p ~/ros_gz/src
cd ~/ros_gz/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2
```

Install dependencies
```
cd ~/ros_gz
sudo rosdep init
```
```
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```
