## Setup cuda 
RMB to setup
[https://docs.nvidia.com/cuda/cuda-installation-guide-linux/](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
## Setup Cuda
Remove cuda

https://stackoverflow.com/questions/56431461/how-to-remove-cuda-completely-from-ubuntu 

Install gcc and extras
```
sudo apt update
sudo apt install build-essential
```

Install cuda
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda
sudo apt-get -y install nvidia-gds
sudo reboot
```

## Install ROS2 Humble Binary + Gazebo Garden from source in Ubuntu 22.04
Endgoal = Simulation test in https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html#prerequisites

1. ROS From Binary
[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html ](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html )

2. Gazebo from Binary
[https://gazebosim.org/docs/garden/install_ubuntu](https://gazebosim.org/docs/garden/install_ubuntu)

3. Ros_Gz Linking Gazebo and ROS
[https://github.com/gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)

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
sudo apt install ros-humble-desktop -y
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt install ros-dev-tools -y
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

## 2. Install Gazebo Garden from binary 
link
Install additional bridges
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt-get install ros-humble-ros-ign-bridge -y
```

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
sudo apt-get update
sudo apt-get install kakoune wget gnupg apt-utils -y
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

sudo apt-get install gz-sim7-cli libgz-transport12-dev libgz-math7-dev libgz-cmake3-dev libignition-cmake2-dev libgz-sim7-dev -y

mkdir -p ros2 && cd ros2
sudo rosdep init
rosdep update
mkdir -p ~/ros2/src && cd ~/ros2/src 
git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git 
export GZ_VERSION=garden
cd ~/ros2
rosdep install -r --from-paths src -i -y --rosdistro humble
source /opt/ros/humble/setup.bash && colcon build
sudo apt-get install gz-garden -y

in bash.rc
source ~/ros2/install/setup.bash
```

## 3. Install ros_gz to link gazebo and ros
```
sudo apt install git
```

```
cd
mkdir -p ~/ros_gz/src
cd ~/ros_gz/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
```


# Test

```
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg
```
```
gz topic -e -t /chatter
```

```
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'" --once
```
```
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan
```
```
rviz2
```
