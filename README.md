# moveit2_workshop
Host packages used in ROS-I ROS2 Manipulation Workshops

## Requirements

From the workspace directory, just above the `src` folder.

```bash
git clone git@github.com:ROS-I-Training/moveit2_workshop.git src

vcs import src --skip-existing --input src/moveit2_workshop/dependencies_rolling.repos

vcs import src --skip-existing --input src/ur5e_cell/ur5e_workcell.repos

source /opt/ros/rolling/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/local_setup,bash
```

## Bringup

To start the perception pipeline and test if Aruco markers work correctly:

```bash
ros2 launch moveit2_workshop_bringup marker_detection.launch.py
```

To start the full demo including marker detection, robot driver, moveit, rviz and demo application:

```bash
ros2 launch moveit2_workshop_bringup full_demo.launch.py
``` 


## Package Description

* **moveit2_workshop**: Metapackage to bind together dependent packages
* **moveit2_workshop_app**: Contains the demo application
* **moveit2_workshop_bringup**: Contains launch and configurations to bringup the demo