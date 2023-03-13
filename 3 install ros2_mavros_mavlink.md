```
sudo apt-get install libasio-dev
sudo apt-get install ros-humble-mavros
```
```
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x ./install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

To run the full combo ROS2 humble, gazebo_garden, ardupilot and mavros
Download the 3 files in this repo and then run it in that directory.
Currently the launch is configured to match the simulation environment so we're using udp://:14550@ based on mavproxy.py
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

```
gz sim -v4 -r iris_runway.sdf
```

```
ros2 launch node.launch
```
