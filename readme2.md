## Install ROS2 Humble/Rolling
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
