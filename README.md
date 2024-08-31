
# D Star Lite Path Planner

## Overview
This repository contains a ROS 2-based path planning system using the D* lite algorithm. It includes nodes for map publishing, dstar_lite, and a action-client for requesting paths.

**Components**: 
1. map_publisher: Publishes a map with obstacles.
2. dstar_lite: Handles path planning requests and publishes the planned path with consideration of dynamic obstacle.
3. dstar_lite_action_client: Requests a path from the dstar_lite node.
4. RViz: Visualization tool for displaying the map and path.


## ROS 2 File structure
```
├── dstar_lite
│   ├── CMakeLists.txt
│   ├── include
│   │   └── dstar_lite
│   │       ├── dstar_lite.hpp
│   │       └── dstar_node.hpp
│   ├── launch
│   │   └── dstar_lite.launch.py
│   ├── package.xml
│   └── src
│       ├── dstar_lite_action_client.cpp
│       ├── dstar_lite.cpp
│       ├── dstar_node.cpp
│       ├── main.cpp
│       └── map_publisher.cpp
├── dstar_lite_interfaces
│   ├── action
│   │   └── Pathfinding.action
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── DStarLiteGoal.msg
│   └── package.xml
├── dstart_lite_planner.rviz
├── images
│   ├── dstar_lite_action_client.png
│   ├── dstar_lite_node.png
│   ├── map_publisher.png
│   ├── path_generated_msg.png
│   └── rviz2.png
└── README.md

```

## Prerequisites
1. Ubuntu 22.04
2. Ros2 Humble

## Setup

##### Open a 4 terminal 

**1. In terminal A**
  ```
  colcon build
  source install/setup.bash
  ros2 run dstar_lite map_publisher
  ```
![Map Publisher](/images/map_publisher.png)

**2. In terminal B**
  ```
  source install/setup.bash
  rviz2
  ```
> **Note:** Rviz should use dstar_lite_planner.rviz file configuration.

**3. In terminal C**
  ```
  source install/setup.bash
  ros2 launch dstar_lite dstar_lite.launch.py
  ```
![Dstar_lite Planner Start](/images/dstar_lite_node_startup.png)

**4. In terminal D**
  ```
  source install/setup.bash
  ros2 run dstar_lite dstar_lite_action_client
```
![Dstar_lite Planner Action Client](/images/dstar_lite_action_client.png)

## Output
1. Dstar_lite planner will server the request came from action-client and will generate the path provided request is valid.![Dstar_lite Planner Result](/images/path_generated_msg.png)

2. Generated path can be visualized in Rviz.    ![Rviz](/images/rviz_output.png)


#### Developer Information

- **Name:** Shailesh Pawar
- **Contact:** shaileshpawar320@gmail.com