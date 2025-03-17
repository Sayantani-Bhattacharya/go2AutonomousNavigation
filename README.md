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

## Tools and References:

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
