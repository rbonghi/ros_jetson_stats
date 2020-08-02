# ROS jetson-stats

A ROS wrapper [jetson-stats](https://github.com/rbonghi/jetson_stats) to ROS where you can read the status of your board via diagnostic messages.

# Installation

1. Check your NVIDIA board have installed jetson-stats otherwise install it
```elm
sudo -H pip install -U jetson-stats
```
2. Clone this repository in your workspace
3. Run catkin_make to locate `ros_jetson_stats` in your workspace

## Setup your launch file

Add in your launch file the `ros_jetson_stats` package following
```xml
<node pkg="ros_jetson_stats" type="jetson_stats.py" name="ros_jetson_stats"/>
```

# Services

You can enable and disable jetson_clocks, set the NV Power Model or change the FAN mode directly from ros service.

* **jetson_clocks** (_ros_jetson_stats/jetson_clocks_)
    * status: Boolean value
* **nvpmodel** (_ros_jetson_stats/nvpmodel_)
    * nvpmodel: NVP Model name, please refer NVIDIA documentation
* **fan** (_ros_jetson_stats/fan_)
    * mode: default, system, manual

# Run the demo

Run the demo following this roslaunch file
```elm
roslaunch ros_jetson_stats jetson_stats.launch
```
Watch your NVIDIA Jetson stats from your runtime_monitor
```elm
rosrun rqt_runtime_monitor rqt_runtime_monitor
```
![runtime_monitor](.github/runtime_monitor.png)