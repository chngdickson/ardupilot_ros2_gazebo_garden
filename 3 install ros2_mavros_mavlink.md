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
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

```
gz sim -v4 -r iris_runway.sdf
```

```
ros2 launch node.launch
```
