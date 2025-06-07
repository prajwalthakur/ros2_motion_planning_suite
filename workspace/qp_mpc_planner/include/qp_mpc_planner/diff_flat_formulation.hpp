#ifndef QP_MPC_PLANNER_DIFF_FLAT_FORMULATION_HPP_
#define QP_MPC_PLANNER_DIFF_FLAT_FORMULATION_HPP_
#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <project_utils/common_utils.hpp>
#include "qp_mpc_planner/planner_utils.hpp"
#include "bernstein_pol.hpp"
#include <eigen-quadprog/QuadProg.h>
#include <eigen-quadprog/eigen_quadprog_api.h>
//obs_poses : num_obs*(3*horizon_length) , 3 reperesents (x,y,yaw) prediction along the horizon
namespace diffflatformulation{

    void init_prob(PlannerParam& );
    void create_prob(const Eigen::ArrayXXf& , const Eigen::Array3f& ,const Eigen::ArrayXXf& , PlannerParam& );

    void solve_prob(PlannerParam&);
    void computeXY(PlannerParam&);
    void continous_collision_avoidance(PlannerParam& );
    void compute_controls(PlannerParam&);
    Eigen::ArrayXXf stack(const Eigen::ArrayXXf & arr1, const Eigen::ArrayXXf & arr2, char ch);
    Eigen::ArrayXXf block_diag(const Eigen::ArrayXXf & arr1, const Eigen::ArrayXXf & arr2);

}
#endif