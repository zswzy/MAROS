# MAROS
A ROS2 package for quick validation and simulation of drone formation control

Multi-UAV Offboard Control Demo using PX4 + ROS2 Jazzy

This repository provides an example of **multi-drone offboard control**
using **PX4 SITL**, **ROS2 Jazzy**, **Micro XRCE-DDS**, **Gazebo**, and
**QGroundControl**.

A default scenario of **1 leader + 4 followers** is included:

-   The **leader UAV** is directly controlled from **QGroundControl**
-   The **followers** run in **Offboard mode** and perform automatic
    formation behavior



## ✨ Features

-   Multi-UAV simulation with **PX4 SITL**
-   Fully working **ROS2 package**: `my_px4_demo`
-   **Micro XRCE-DDS** for PX4 ↔ ROS2 communication
-   Ready-to-use startup script `autostart.sh`
-   Supports formation control with configurable UAV numbers
-   Supports user-defined formation shape and MAS numbers/topology
-   Multi-process running



## 📦 Dependencies

Please make sure the following components are installed before using
this repository.

### 1. PX4, Gazebo, QGroundControl

Official installation guides:

-   **PX4 Autopilot (including SITL and Gazebo)**\
    https://docs.px4.io/main/en/
-   **QGroundControl**\
    https://qgroundcontrol.com/

### 2. ROS2 Jazzy

https://docs.ros.org/en/jazzy/Installation.html

### 3. Micro XRCE-DDS (required for PX4--ROS2 communication)

https://docs.px4.io/main/en/middleware/uxrce_dds



## 📂 Repository Structure & Placement

1.  Place the folder **`my_px4_demo`** from this repository into your
    ROS2 workspace:

        ~/ros2_ws/src/my_px4_demo

2.  Place **`autostart.sh`** into your home directory:

        ~/autostart.sh

3.  Download the official PX4 ROS2 message package **px4_msgs** into
    your workspace:

        ~/ros2_ws/src/px4_msgs

    GitHub repository:\
    https://github.com/PX4/px4_msgs



## 🔨 Build the ROS2 Packages

Inside your ROS2 workspace:

``` bash
cd ~/ros2_ws
colcon build --packages-select my_px4_demo
colcon build --packages-select px4_msgs
```





## 🚁 Run the Multi-UAV Simulation

### 1. Start PX4 SITL + Gazebo (multi-vehicle)

``` bash
./autostart.sh
```

This script automatically launches multiple PX4 instances and the Gazebo
world.

After startup, open **QGroundControl** to connect to the leader UAV.



## ▶️ Launch the ROS2 Demo

In a separate terminal:

Source the workspace:

``` bash
source install/setup.bash
```

Launch the ROS2 package

``` bash
ros2 launch my_px4_demo launch_etm_demo.launch.py
```



## 📌 Default Scenario

-   Total **5 UAVs**
-   **1 Leader (ID 0)** --- controlled via QGroundControl\
-   **4 Followers (ID 1--4)** --- automatically controlled in **Offboard
    mode**

You can modify the drone count or behavior in:\
`formation_node.py`.



## 📄 License


## Reference

[1] Z. Wang, M. Chadli, and S. X. Ding, “A dynamic event-triggered approach for observer-based formation control of multi-agent systems with designable inter-event time,” Systems & Control Letters, vol. 195, p. 105970, Nov. 2024, doi: 10.1016/j.sysconle.2024.105970.



