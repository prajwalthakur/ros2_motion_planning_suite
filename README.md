# Motion Planning Algorithms in ROS2

This repository contains the various motion planning algorthims , including obstacle avoidance strategies in ros2 jazzy

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