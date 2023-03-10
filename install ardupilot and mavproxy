## 1. Install Ardupilot

```
sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
```

clone repo
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

configure git to use https
```
git config --global url.https://.insteadOf git://
```

Install on Ubuntu
```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Install MavProxy
```
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
```

Reload the profile
```
. ~/.profile
```

Add these lines to bash.rc file in home
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
export PATH="$PATH:$HOME/.local/bin"
```


## Plugin to connect Ardupilot Gazebo Installation
```
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
```
