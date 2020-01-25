# ROS jetson-stats

A wrapper from jetson stats to ROS

# Installation

1. Check your NVIDIA board have insalled jetson-stats otherwise install it
```elm
sudo -H pip install -U jetson-stats
```
2. Clone this repository in your workspace
3. Run catkin_make to locate `ros_jetson_stats` in your workspace

# Run the demo

Run the demo following this roslaunch file
```elm
roslaunch ros_jetson_stats jetson_stats.launch
```