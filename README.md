## 1 Install ROS2 Humble/Rolling in Ubuntu 22.04
ROS From Binary
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html 
Gazebo from Source
https://github.com/gazebosim/ros_gz/tree/humble#from-source
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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install packages
```
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```


Create workspace and clone all repos
```
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```

Install dependencies with Rosdep
```
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

Build the installed files
```
cd ~/ros2_humble/
colcon build --symlink-install
```

Go to bash.rc file in your ubuntu Home, add this line
```
source ~/ros2_humble/install/local_setup.bash
```

Test examples
Talker
``` 
ros2 run demo_nodes_cpp talker
```
Listener
```
ros2 run demo_nodes_py listener
```

## Install Gazebo Garden from source 
link
```https://gazebosim.org/docs/garden/install_ubuntu_src```

vcstool and colcon 
```
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

Get sources and download
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

Building the Gazebo Libraries
```
cd ~/gazebo/
colcon graph
colcon build --cmake-args ' -DBUILD_TESTING=OFF' ' -DCMAKE_BUILD_TYPE=Debug' --merge-install
```

In the file bash.rc in Home, add this line
```
source ~/gazebo/install/setup.bash
```

### Link Ros to Gazebo-Garden
```
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
```

check that resolve works
```
rosdep resolve gz-garden
```

```
export GZ_VERSION=garden
```

compile ros_gz
```
cd ~/gazebo/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2
```

Install ros_gz
```
cd ~/gazebo
rosdep install -r --from-paths src -i -y --rosdistro humble
```

Build the workspace
```
cd ~/gazebo
colcon build
```
