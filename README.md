# ROS2 Package for Autnomous Navigation of Unitree GO2.

Author: Sayantani Bhattacharya

## Project Overview:

Navigating complex and dangerous terrains autonomously, this system equips the Unitree GO2 quadruped with advanced exploration capabilities. Designed for unstructured environments, it integrates real-time localization, mapping, and intelligent navigation to move efficiently through forests, rubble, or unknown spaces. The robot autonomously detects frontiers, selects optimal paths, and avoids obstacles, ensuring safe and effective traversal. With a state-machine-driven approach and real-time sensor feedback, it dynamically adapts to its surroundings, making decisions on the fly. Whether mapping unknown areas or assisting in hazardous missions, this system pushes the boundaries of quadrupedal autonomy, bridging the gap between robotic perception and intelligent motion.

<p align="center">
  <img src="/images/go2Outside.gif" alt="Alt text" width="800"/>
</p>

## Block Diagram:

<p align="center">
  <img src="/images/go2.png" alt="Alt text" width="800"/>
</p>

## Repository Structure:

There are three main packages in the form of nested submodules ensuring code clarity, efficiency, and adaptability. 
At its core, high-level controls interact fluidly with the GO2 ROS2 SDK, bridging the gap between perception and action. The manual navigation module empowers users with precise control, featuring robust state publishing, real-time transform broadcasting, and LiDAR-driven mapping for dynamic visualization in RViz. Building upon this foundation, the autonomous navigation package takes exploration to the next level—leveraging nearest-frontier strategies and a state-machine-driven approach to efficiently map unknown terrains.


</br>
The following are the abstaction layers, starting from the lowest layer:

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


## Procedure to run:

Current(will try to reduce the steps):

### Terminal 0 [Prepping Environment]:
    ./setup.sh  # In the unitree_ros submodule after connecting to GO2, as mentioned in its ReadMe.
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

### Terminal 1:  [unitree_go2_nav]  -> run in [main_pkg]
The base launch file for transforms, robot-state, slam and manual navigation.
      
      ros2 launch unitree_go2_nav navigation.launch.py


### Terminal 2: [main_pkg]
This subscripes to map data by rtabmap pkg, and unitree topics, And solves for exploration goals.

    ros2 run go2_exploration go2Exploration 


### Terminal 3: [unitree_go2_nav]
This subscribes to nav pkg, and publishes to the high level controller of GO2.
       
       ros2 run unitree_go2_nav navToPose


### Terminal 4: [unitree_ros]
This publishes to Unitree APIs, final abstaction layer.

    ./install/unitree_ros2_example/bin/high_level_ctrl 


### Terminal 5: 
Trigger for GO2 to start exploring.

    ros2 service call /explore_start go2_exploration_interfaces/srv/Empty


## Troubleshooting:
1. When building unitree_go2_nav for the first time, or after colcon clean,always follow:
       
       colcon build --packages-select cyclonedds
       source /opt/ros/jazzy/setup.bash  # [dont have this in bashrc if you are following the instruction of unitree_ros2, and using cyclonedds as middleware]
       colcon build
       source install/setup.bash


## Tools:

### Hardware:
  - Unitree GO2.
  - 4D Lidar.
  - System76 for on-board computation.
  - System76 for real-time visualisation.
  - Ethernet cable.

### Software : 
  - C++ Jazzy
  - ROS2
  - Python
  - Unitree SDK GO2
  - ROS2 pkgs: RTabMap and Nav2.
