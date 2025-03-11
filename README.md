# Motion Planning and Obstacle Avoidance Algorithms in ROS2 and C++
A C++ based ROS2 software stack for rapid testing of motion planning, obstacle avoidance, and filtering techniques.
1. A modularized package supporting various vehicle dynamics/kinematics models. 
2. The vehicle class is implemented as a shared/static library and leverages CppAD for automatic differentiation, enabling seamless integration with existing planning algorithms.

```
.
├── eigen3_cmake_module
│   ├── CHANGELOG.rst
│   ├── cmake
│   │   └── Modules
│   │       └── FindEigen3.cmake
│   ├── CMakeLists.txt
│   ├── CONTRIBUTING.md
│   ├── eigen3_cmake_module-extras.cmake
│   ├── LICENSE
│   ├── package.xml
│   └── README.md
├── project_utils
│   ├── CMakeLists.txt
│   ├── config
│   │   └── vehicle_params.yaml
│   ├── include
│   │   └── project_utils
│   │       ├── integrator_class.hpp
│   │       └── vehicle_class.hpp
│   ├── msg
│   │   └── EigenVector.msg
│   ├── package.xml
│   └── src
│       ├── integrator_class.cpp
│       └── vehicle_class.cpp
├── README_tree.md
├── test_jax.py
└── vehicle_interface
    ├── CMakeLists.txt
    ├── include
    │   └── vehicle_interface
    │       └── vehicle_interface_class.hpp
    ├── launch
    │   └── vehicle_interface.launch.py
    ├── package.xml
    └── src
        ├── mpc_node.cpp
        ├── vehicle_interface_class.cpp
        └── vehicle_interface_node.cpp

14 directories, 25 files
```


This repository contains the various motion planning algorthims , including obstacle avoidance strategies in ros2 humble.

List of Vehicles
1. Single Track Ackerman Steering Kinematics Model


List of Motion Planning Algorithms

1. Quadratic Programming (QP) Based Model Predictive Control
2. QP based Model Predictive Contouring Control
3. Model Predictive Path Integral Contol (MPPI) based Path Tracking Algorithm
4. Frenet Frame Based Non-Linear MPC 

List of Obstacle Avoidance Algorithm
1. Corridor Based Obstacle Avoidance ( can be used with 1,2,3 motion planning )
2 . Control Barrier Function (CBF) , only be used with non-linear MPC 

# Docker Image Installation 
1. Change the root directory name in `scripts/deploy/base.sh` 
2. From root of the directory  run  `./scripts/build/build.sh`
After above two steps the Docker Image with the name of **mp_ros2** would have been created

# To run the docker Conatiner

3. From root of the directory run `./scripts/deploy/devel.sh`


## License
MIT  


## Use Case
I initiated this project to independently study algorithms and software development for autonomous vehicle systems. This repository is also available for your personal use in studying, education, research, or development.

If this project supports your work or contributes to your tasks, please feel free to inform me by starring the repository.


## Contribution
Any contribution by creating an issue or sending a pull request is welcome!! 
<!-- Please check [this document about how to contribute](/HOWTOCONTRIBUTE.md).   -->


## Update-1
## vehicle Interface node has been implemented



## Author
[Prajwal Thakur](https://github.com/prajwalthakur) 

