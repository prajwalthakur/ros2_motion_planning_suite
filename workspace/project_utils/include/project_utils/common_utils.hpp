#ifndef PROJECT_UTILS_COMMON_UTILS_HPP_
#define PROJECT_UTILS_COMMON_UTILS_HPP_
#pragma once 
#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> MapArrayXfRow;
typedef Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> MapMatrixfRow;
typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXfRow;
using AXXf = Eigen::ArrayXXf;
using AXf = Eigen::ArrayXf;       // column vector
void load_map(std::string, std::vector<float> &, MapArrayXfRow &);
static int NUMCOL =3;
#endif