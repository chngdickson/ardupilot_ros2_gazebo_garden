# ardupilot_ros_noetic_gazebo11
### Download Boost
```
sudo apt remove libboost-all-dev -y
sudo apt autoremove -y
sudo apt apt purge -y libboost*
sudo apt install libboost-all-dev
sudo apt update && sudo apt upgrade
```
### Install the colmap
```
sudo apt-get install \
    git \
    cmake \
    ninja-build \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libflann-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsqlite3-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libceres-dev
```

```
cd 
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build
cd build
cmake .. -GNinja
ninja
sudo ninja install
``` 
### Download Ardupilot
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
git config --global url.https://.insteadOf git://
Tools/environment_install/install-prereqs-ubuntu.sh -y

. ~/.profile
```

In your bash.rc
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Reload bash.rc
```
. ~/.bashrc
```

### Install Ardupilot ROS noetic
```
source ~/ardupilot/Tools/environment_install/install-ROS-ubuntu.sh
```


