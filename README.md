# ROS2 Package for Autnomous Navigation of Unitree GO2.

Author: Sayantani Bhattacharya

## Project Overview

To build a swarm of quadrupeds, capable of collaborative exploration missions, in hazardous/dangerous terrains.
Quadrupeds are ideal for moving in uneven areas like mines, forests, etc and thus chosen for the task. The aim is to 
build a system that performs localization, mapping, navigation and attention detection (can be a human/hazard prone area, 
depending on the task). Each quadruped would have these capabilities, and would also have the ability to merge the 
information from the agents in its vicinity (decentralized), to make a more robust system, and explore an area at the 
fastest possible speed. Agents would be aware of each-other’s presence and work in collaboration with each other.

## Block diagram

<!-- 1. Complete system:</br>
<p align="right">
  <img src="/images/system_block.png" alt="Alt text" width="700"/>
</p> -->

<!-- 2. The Block diagram:</br>
    (subject to modifications) </br> -->
<p align="right">
  <img src="/images/go2.png" alt="Alt text" width="700"/>
</p>

## Repository Structure

There are 3 main packages, in the for of nested submodules to maintain clarity of code. 
The following are the abstaction layer, starting from the lowest layer.

- Implemented high level controls that interacts with the Unitree GO2 ROS2 SDK.

- Implemented manual navigation package with following features in ROS2 Jazzy and C++:

    1. Integrated with high level control pkg to operate on real robot.
    2. Transform publishers.
    3. Robot State publishers.
    4. Rtabmap pkgs generating odom tf and occupancy grid using GO2’s 4d Lidar.
    5. Rviz visulaizations with modified URDF.
    6. Nav-to-pose to enable high level control with the GO2’s APIs.
    7. Nav2 based manual goal subscription.
    8. Footprint based obstacle avoidance.

- Implemented autonomous navigation package with following features in ROS2 Jazzy and C++:

    1. Integrated with the manual nav package to operate on real robot.
    2. Nearest frontier based exploration.
    3. State machine based goal assignment.
    4. Service calls to trigger start, and emergency stop.
    5. Markers for real-time visualization.


## Tools:

### Hardware:
  - Unitree GO1-GO2.
  - Zed Camera/ Lidar (Need to confirm).
  - Jetson Orin Nano.
  - buck-convertor (24V->12V).
  - 3D print the mount for unitree.
  - Display port adapters for Jetson.
  - Ethernet cable for initial testing with unitree sdk.
  - Micro SD cards.

### Software : 
  - C++
  - ROS2
  - Python
  - Unitree SDK - GO1 and GO2
  - Slam, RTabMap, gmapping and Nav2 pkg
