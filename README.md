## Install ROS2 Humble Binary + Gazebo Garden from source in Ubuntu 22.04

RMB to setup
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/

ROS From Binary
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 

Gazebo from Source
https://github.com/gazebosim/ros_gz/tree/humble#from-source

## Install ROS2 Humble
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

## Install Gazebo Garden from source 
link
```https://gazebosim.org/docs/garden/install_ubuntu_src```
Export GZ Version, used for Gazebo version the ROS Would like to compile against
```
source /opt/ros/humble/setup.bash
export GZ_VERSION=garden
```

vcstool and colcon 
```
sudo apt install python3-pip wget lsb-release gnupg curl
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions
```

Add git
```
sudo apt-get install git
```

Create gazebo Workspace
```
mkdir -p ~/gazebo/src
cd ~/gazebo/src
```

GET ROS SOURCES and DOWNLOAD
```
git clone https://github.com/gazebosim/ros_gz.git -b ros2

```
Get Gazebo sources and download
```
wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml
vcs import < collection-garden.yaml
```

More dependencies
```
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update 
sudo apt-get upgrade --fix-missing -y
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
```


### Link Gazebo with ros_gz
```
cd ~/gazebo
sudo rosdep init
rosdep install -r --from-paths src -i -y --rosdistro humble
```

Build the workspace
```
cd ~/gazebo
colcon build
```

Source the workspace in bash.rc
```
source ~/gazebo/install/setup.bash
```
