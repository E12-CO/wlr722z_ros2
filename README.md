# wlr722z_ros2

ROS2 node for Vanjee WLR-722Z 16 lines 3D LiDAR.

Build :
-
```colcon build --symlink-install --packages-select wlr722z_ros2```

Run :
-
```ros2 launch wlr722z_ros2 wlr722z.launch.py```

# TODO
- Add the parameter query
- Improve the communication side to reduce error rate. Possibly need 25MHz XTAL on STM32 to match 3.125MBaud rate of th Lidar.
