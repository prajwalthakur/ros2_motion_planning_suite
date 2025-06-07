/**
 * @file spline_traj_planner.hpp
 * @brief Utility header only class to compute the smooth spline trajectory
 */
#ifndef QP_MPC_PLANNER_SPLINE_TRAJ_PLANNER_HPP_
#define QP_MPC_PLANNER_SPLINE_TRAJ_PLANNER_HPP_
    #pragma once 
    #include <iostream>
    #include <eigen3/Eigen/Dense>
    #include <vector>
    #include "yaml-cpp/yaml.h"
    typedef Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>> MapMatrixCol;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRow;
    using AXXf = Eigen::ArrayXXf;
    using AXf = Eigen::ArrayXf;       // column vector

    namespace bernsteinpol{

        /**
         * @struct five_var
         * @brief struct for storing bernstein Matrix, 0th derivative to 4th derivative.
         */
        struct five_var
        {
            AXXf a, b, c, d, e;
        }; 

        five_var bernsteinCoeffOrder10(float , float , float , AXXf , int );
        five_var computeBernstein(float , float  );
        AXXf stack(AXXf& arr1, AXXf& arr2, char ch);
        AXXf reshape(AXXf x, uint32_t r, uint32_t c);
        AXXf block_diag(AXXf arr1, AXXf arr2);
        float binomialCoeff(float , float );
    }
#endif