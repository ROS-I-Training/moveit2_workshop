# moveit2_workshop
Host packages used in ROS-I ROS2 Manipulation Workshops

## Requirements

From the workspace directory, just above the `src` folder.

```bash
git clone git@github.com:ROS-I-Training/moveit2_workshop.git src

sudo apt install python3-vcstool

vcs import src --skip-existing --input src/moveit2_workshop/dependencies_rolling.repos

vcs import src --skip-existing --input src/ur5e_cell/ur5e_workcell.repos

source /opt/ros/rolling/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/local_setup,bash
```

## Package Description

* **moveit2_workshop**: Metapackage to bind together dependent packages
* **moveit2_workshop_app**: Contains the demo application
* **moveit2_workshop_bringup**: Contains launch and configurations to bringup the demo

## Bringup

### Devices

To start only the perception pipeline and test if Aruco markers work correctly:
```bash
ros2 launch moveit2_workshop_bringup marker_detection.launch.py
```

### Panda Demo Application

To start demos of the applications on Panda rviz only (no real robot)

Simple app:   
```bash
ros2 launch moveit2_workshop_bringup dummy_app_simple.launch.py
```
Marker app (has static TFs for the markers in the launch):
```bash
ros2 launch moveit2_workshop_bringup dummy_app_marker.launch.py
```
### UR5e Cell

To start the demo on the IPA326 UR5e Cell:

```bash
ros2 launch moveit2_workshop_bringup full_demo.launch.py
``` 

## Demo Applications

### Simple Application

`moveit2_workshop_app/src/app_simple.cpp`

A very simple application that moves to two parametrised poses in cartesian space. These pose values are read from a parameter file, see below.

### Marker Application

`moveit2_workshop_app/src/app_marker.cpp`

Reads poses of two aruco markers from the TF tree and moves above each in a sequence. The aruco pose detection needs to be running for this to work.

## Configuration

### Configuring the marker detection
Following configs are placed in the `moveit2_workshop_bringup/config/marker_detection` folder:

* **aruco_node_params.yaml**: Params used by `aruco_ros2` node, including camera topics, marker size and TF frame name.
* **camera_node_params.yaml**: Params used by `usb_cam` node, including camera source, image properties and link to calibration.
* **camera_info.yaml**: Camera calibration data, published in the `camera_info` topics by the `usb_cam` node.
* **camera_tf.yaml**: Contains positional calibration of the camera mount. Update if camera is moved.

### Configuring the applications
For the Panda demo, configs are placed in `moveit2_workshop_bringup/config/app/panda`.
For the UR5e Cell, configs are placed in `moveit2_workshop_bringup/config/app/ur5e_cell`.

* **app_simple.yaml**: Params to set the planning group, and "pick" and "place" pose values for the `app_simple` node. The poses are defined w.r.t the planning group's base_link frame.
* **app_marker.yaml**: Params to set the planning group, and names of the aruco marker frames for the `app_marker` node. 
