# CppND-Capstone-Project: Robot Simulation

## About The Project

This project simulates robot path planning process using ROS and Gazebo simulation environment, featuring a visulization of the expanding process of the algorithm. It helps the algorithm learner to understand intuitively how the path planning algorithm really works. Currently only A* algorithm implemented, more such as Dijkstra, can be added so that performance between them can be compared.

This Project inspired by XXX, which mainly buidt in Python. https://github.com/rfzeg/path_planning_intro

## Dependencies for Running Locally

- ROS ([Noetic][ros-noetic-installation] for Ubuntu 20.04 or [Kinetic][ros-kinetic-installation] for Ubuntu 16.04)
  
  ROS **Desktop-Full** version is recommended since we need gazebo as simulator and Rviz as visulizer

- [catkin_tools][catkin-tools-doc] - Optional
  
  Installation: make sure ROS is installed on your system (or ROS repo is added into the software source)
  ```bash
  python-catkin-tools
  ```

## Installation

Step 1: Initialize a catkin working space

```bash
mkdir CppND_ws    #take any name you like
cd CppND_ws
mkdir src
catkin init
```

Step 2: Clone the package

```bash
cd src
git clone https://github.com/vacuum136/CppND-Capstone.git
```

Step 3: build the package

```bash
catkin build
```


> **Warning:** This command will take a while because it inserts ~3M rows in the db and [creates indexes to perform efficient searches](https://rdkit.org/docs/Cartridge.html), a rough estimate being between 15 minutes and an hour.

Lastly, start the CppND-Capstone service:

```bash
docker-compose up service
```

Once these commands are ran, you are ready to test the endpoints at http://localhost:8000/swagger/ui .

<!-- USAGE EXAMPLES -->
## Usage

See usage through Swagger demo here https://drive.google.com/file/d/1FKh3-N4KWvVEz7NPFmMDALWu-BzYbaHP/view?usp=sharing .

<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/vacuum136/CppND-Capstone/issues) for a list of proposed features (and known issues).

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Make sure to format your code properly with `make format`.

<!-- LICENSE -->
## License

Distributed under the Apache-2.0 License. See `LICENSE` for more information.



<!-- CONTACT -->
## Contact

Michel ML - [@vacuum136](https://github.com/vacuum136) - vacuum136@gmail.com

Project Link: [https://github.com/vacuum136/CppND-Capstone](https://github.com/vacuum136/CppND-Capstone)

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

Thank you to all contributors of libraries and tools used in this project.

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/vacuum136/CppND-Capstone.svg?style=for-the-badge
[contributors-url]: https://github.com/vacuum136/CppND-Capstone/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/vacuum136/CppND-Capstone.svg?style=for-the-badge
[forks-url]: https://github.com/vacuum136/CppND-Capstone/network/members
[stars-shield]: https://img.shields.io/github/stars/vacuum136/CppND-Capstone.svg?style=for-the-badge
[stars-url]: https://github.com/vacuum136/CppND-Capstone/stargazers
[issues-shield]: https://img.shields.io/github/issues/vacuum136/CppND-Capstone.svg?style=for-the-badge
[issues-url]: https://github.com/vacuum136/CppND-Capstone/issues
[license-shield]: https://img.shields.io/github/license/vacuum136/CppND-Capstone.svg?style=for-the-badge
[license-url]: https://github.com/vacuum136/CppND-Capstone/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/michelmoreau1/
[product-screenshot]: CppND-Capstone.png

[catkin-tools-doc]: https://catkin-tools.readthedocs.io/en/latest/installing.html
[ros-noetic-installation]: http://wiki.ros.org/noetic/Installation/Ubuntu
[ros-kinetic-installation]: http://wiki.ros.org/kinetic/Installation/Ubuntu