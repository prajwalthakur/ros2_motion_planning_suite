#ifndef QP_MPC_PLANNER_DIFF_DRIVE_FORMULATION_HPP_
#define QP_MPC_PLANNER_DIFF_DRIVE_FORMULATION_HPP_
#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <project_utils/common_utils.hpp>
#include "qp_mpc_planner/planner_utils.hpp"
#include "bernstein_pol.hpp"
//obs_poses : num_obs*(3*horizon_length) , 3 reperesents (x,y,yaw) prediction along the horizon

void init_prob(PlannerParam& );
void create_prob(const Eigen::ArrayXXf& , const Eigen::Array3f& ,const Eigen::ArrayXXf& , PlannerParam& );

void solve_prob(PlannerParam&);
void computeXY(PlannerParam&);
void continous_collision_avoidance(PlannerParam& );
#endif