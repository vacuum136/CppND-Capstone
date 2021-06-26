<h1>CppND-Capstone-Project: Robot Simulation</h1>

- [1. About The Project](#1-about-the-project)
- [2. Dependencies](#2-dependencies)
- [3. How To Build](#3-how-to-build)
- [4. Run Simulation](#4-run-simulation)
- [5. Rubric Points](#5-rubric-points)
- [6. Acknowledgements](#6-acknowledgements)

## 1. About The Project

This project simulates robot path planning process using ROS and Gazebo simulation environment, featuring a visulization of the expanding process of the algorithm. It helps the algorithm learner to understand intuitively how the path planning algorithm really works. Currently only A* algorithm implemented, more such as Dijkstra, can be added so that performance between them can be compared.


## 2. Dependencies

- ROS ([Noetic][ros-noetic-installation] for Ubuntu 20.04 or [Kinetic][ros-kinetic-installation] for Ubuntu 16.04)
  
  ROS **Desktop-Full** version is recommended since we need gazebo as simulator and Rviz as visulizer

- [catkin_tools][catkin-tools-doc] - Optional
  
  Installation: make sure ROS is installed on your system (or ROS repo is added into the software source)
  ```bash
  python-catkin-tools
  ```

## 3. How To Build

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

## 4. Run Simulation

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

## 5. Rubric Points  

> Loops, Functions, I/O  
> - The project demonstrates an understanding of C++ functions and control structures.

- while loop: path_planner.cpp line 23-60
- for loop: path_planner.cpp line 38-58
- do-while loop: path_planner.cpp line 66-70

> Object Oriented Programming  
> - The project uses Object Oriented Programming techniques.  
> - Classes use appropriate access specifiers for class members.
> - Class constructors utilize member initialization lists. 
> - Classes abstract implementation details from their interfaces.
> - Classes encapsulate behavior

> Memory Management  
> - The project makes use of references in function declarations. 
> - The project uses move semantics to move data, instead of copying it, where possible.
> - The project uses smart pointers instead of raw pointers.

## 6. Acknowledgements

- This Project inspired by the path planning tutorial by [rfzeg](https://github.com/rfzeg/path_planning_intro), which basically built with Python.
- Thanks [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) for the robot model.




[catkin-tools-doc]: https://catkin-tools.readthedocs.io/en/latest/installing.html
[ros-noetic-installation]: http://wiki.ros.org/noetic/Installation/Ubuntu
[ros-kinetic-installation]: http://wiki.ros.org/kinetic/Installation/Ubuntu