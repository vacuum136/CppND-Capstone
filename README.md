<h1>CppND-Capstone-Project: Robot Simulation</h1>

- [1. About The Project](#1-about-the-project)
- [2. Main Components](#2-main-components)
- [3. Dependencies](#3-dependencies)
- [4. How To Build](#4-how-to-build)
- [5. Run Simulation](#5-run-simulation)
- [6. Rubric Points](#6-rubric-points)
- [7. Acknowledgements](#7-acknowledgements)

## 1. About The Project

This project simulates robot path planning process using ROS and Gazebo simulation environment, featuring a visulization of the expanding process of the algorithm. It helps the algorithm learner to understand intuitively how the path planning algorithm really works. Currently only A* algorithm implemented, more such as Dijkstra, can be added so that performance between them can be compared.

## 2. Main Components

**robot_simulation**: the package that my contribution focus on in this project  
  - path_planner_server: a ROS service server that response with a path after getting the request of a global path from the client
  - path_planner: called by the server and plan the path
  - expand_visual: visualize the expanding process

**pp_msgs**: a customized ROS srv file.  
**srv_client_plugin**: A ROS global planner that creates service request for a plan and forwards the response to the move_base global planner module.  
**turtlebot3_description**: a robot model.  

## 3. Dependencies

- ROS ([Noetic][ros-noetic-installation] for Ubuntu 20.04 or [Kinetic][ros-kinetic-installation] for Ubuntu 16.04)
  
  ROS **Desktop-Full** version is recommended since we need gazebo as simulator and Rviz as visulizer

- [catkin_tools][catkin-tools-doc] - Optional
  
  Installation: make sure ROS is installed on your system (or ROS repo is added into the software source)
  ```bash
  python-catkin-tools
  ```

## 4. How To Build

- Step 1: Initialize a catkin working space

  ```bash
  mkdir CppND_ws    #take any name you like
  cd CppND_ws
  mkdir src
  catkin init
  ```

- Step 2: Clone the package

  ```bash
  cd src
  # for ROS kinetic
  git clone -b kinetic https://github.com/vacuum136/CppND-Capstone.git

  # for ROS noetic
  git clone https://github.com/vacuum136/CppND-Capstone.git
  ```

- Step 3: build the package

  ```bash
  catkin build
  ```

## 5. Run Simulation

> **Notice**: make sure you've set ROS's evironment properly and add the CppND workspace into the ROS environment  
> Option 1: set everytime you launch a terminal
> ```bash
> # Assuming you're using bash shell
> source /opt/ros/[ros-version]/setup.bash
> source .../CppND_ws/devel/setup.bash
> ```
> Option 2: put the above two line into the .bashrc file and save.

Step 1: launch gazebo simulator

```bash
roslaunch robot_simulation robot_gazebo.launch
```

Step 2: launch path planner node
```bash
roslaunch robot_simulation robot_planner.launch
``` 

Then you can simulate the path planning in Rviz. Using the "2D Navi Goal" to set the goal for robot. Once the goal is set, a visualized expanding process will start. After it reachs the goal, the robot will then move along with the founded path to the goal. 

## 6. Rubric Points  

> Loops, Functions, I/O  
> - The project demonstrates an understanding of C++ functions and control structures.

- while loop: path_planner.cpp line 20-60
- for loop: path_planner.cpp line 39-59
- do-while loop: path_planner.cpp line 66-70

> Object Oriented Programming  
> - The project uses Object Oriented Programming techniques. 
> - Classes use appropriate access specifiers for class members.
> - Class constructors utilize member initialization lists. 
> - Classes abstract implementation details from their interfaces.
> - Classes encapsulate behavior

- see class PathPlanner and ExpandVisual
- see publich and private members in class PathPlaner and ExpendVisual
- initialization lists see PathPlanner Constructor and ExpandVisual Constructor
- PathPlanner::getPath(), see path_planner.h line 66

> Memory Management  
> - The project makes use of references in function declarations. 
> - The project uses move semantics to move data, instead of copying it, where possible.
> - The project uses smart pointers instead of raw pointers.

- one of the PathPlanner's contructor's parameter is a reference to the costmap
- using move semantics when pushing back founed neighbor cell into open list. See path_planner.cpp line 56
- sensor_msg/CloudPoint2 is a share_point of point cloud vector. See expand_visual.cpp line 32.

## 7. Acknowledgements

- This Project inspired by the path planning tutorial by [rfzeg](https://github.com/rfzeg/path_planning_intro), which basically built with Python.
- Thanks [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) for the robot model.




[catkin-tools-doc]: https://catkin-tools.readthedocs.io/en/latest/installing.html
[ros-noetic-installation]: http://wiki.ros.org/noetic/Installation/Ubuntu
[ros-kinetic-installation]: http://wiki.ros.org/kinetic/Installation/Ubuntu