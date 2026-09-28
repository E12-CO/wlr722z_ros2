# wlr722z_ros2

ROS2 node for Vanjee WLR-722Z 16 lines 3D LiDAR.

Build :
-
```colcon build --symlink-install --packages-select wlr722z_ros2```

Run :
-
```ros2 launch wlr722z_ros2 wlr722z.launch.py```

# TODO
- Add parameter file
- Slow down the pointcloud publish rate to 10s Hz range
- Add the checksum checking
- Add the parameter query
