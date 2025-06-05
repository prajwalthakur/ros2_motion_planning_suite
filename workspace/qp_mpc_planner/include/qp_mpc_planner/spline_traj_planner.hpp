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
AXXf stack(AXXf arr1, AXXf arr2, char ch);
AXXf reshape(AXXf x, uint32_t r, uint32_t c);
AXXf block_diag(AXXf arr1, AXXf arr2);



float binomialCoeff(float n, float k){
    if(k==0 || k==n){return 1;}
    return binomialCoeff(n-1,k-1) + binomialCoeff(n-1,k);
}
/**
 * @brief compute Polynomial Matrix
 * @param n polynomial-order.
 * @param t_min starting time.
 * @param t_max end time.
 * @param t_actual vector of time, where t_min = t_actual(0) and t_max = t_actual(-1).
 * @param num horizon-steps
 * @return struct five_var : bernstein Matrices from (order 0 to 4th order) 
 */
five_var bernsteinCoeffOrder10(float n, float t_min, float t_max, AXXf t_actual, int num){

    five_var s;
    float l = t_max - t_min;
    AXXf t = (t_actual-t_min)/l;   // scaling the t to (0 to 1)
    AXXf P(num,(int)n+1), Pdot(num,(int)n+1), Pddot(num,(int)n+1); // n is the order of the polynominal , n order has n+1 coefficient
    AXXf Pdddot(num,(int)n+1),Pddddot(num,(int)n+1);
    // P
    for(int i=0;i<=n;++i){
    P.col(i) = binomialCoeff(n,i)*pow(1-t,n-i)*pow(t,i);
    }

    // Pdot
    Pdot.col(0) = -10.0 * pow(-t + 1, 9);
    Pdot.col(1) = -90.0 * t * pow(-t + 1, 8) + 10.0 * pow(-t + 1, 9);
    Pdot.col(2) = -360.0 * pow(t, 2) * pow(-t + 1, 7) + 90.0 * t * pow(-t + 1, 8);
    Pdot.col(3) = -840.0 * pow(t, 3) * pow(-t + 1, 6) + 360.0 * pow(t, 2) * pow(-t + 1, 7);
    Pdot.col(4) = -1260.0 * pow(t, 4) * pow(-t + 1, 5) + 840.0 * pow(t, 3) * pow(-t + 1, 6);
    Pdot.col(5) = -1260.0 * pow(t, 5) * pow(-t + 1, 4) + 1260.0 * pow(t, 4) * pow(-t + 1, 5);
    Pdot.col(6) = -840.0 * pow(t, 6) * pow(-t + 1, 3) + 1260.0 * pow(t, 5) * pow(-t + 1, 4);
    Pdot.col(7) = -360.0 * pow(t, 7) * pow(-t + 1, 2) + 840.0 * pow(t, 6) * pow(-t + 1, 3);
    Pdot.col(8) = 45.0 * pow(t, 8) * (2 * t - 2) + 360.0 * pow(t, 7) * pow(-t + 1, 2);
    Pdot.col(9) = -10.0 * pow(t, 9) + 9 * pow(t, 8) * (-10.0 * t + 10.0);
    Pdot.col(10) = 10.0 * pow(t, 9);

    Pddot.col(0) = 90.0 * pow(-t + 1, 8.0);
    Pddot.col(1) = 720.0 * t * pow(-t + 1, 7) - 180.0 * pow(-t + 1, 8);
    Pddot.col(2) = 2520.0 * pow(t, 2) * pow(-t + 1, 6) - 1440.0 * t * pow(-t + 1, 7) + 90.0 * pow(-t + 1, 8);
    Pddot.col(3) = 5040.0 * pow(t, 3) * pow(-t + 1, 5) - 5040.0 * pow(t, 2) * pow(-t + 1, 6) + 720.0 * t * pow(-t + 1, 7);
    Pddot.col(4) = 6300.0 * pow(t, 4) * pow(-t + 1, 4) - 10080.0 * pow(t, 3) * pow(-t + 1, 5) + 2520.0 * pow(t, 2) * pow(-t + 1, 6);
    Pddot.col(5) = 5040.0 * pow(t, 5) * pow(-t + 1, 3) - 12600.0 * pow(t, 4) * pow(-t + 1, 4) + 5040.0 * pow(t, 3) * pow(-t + 1, 5);
    Pddot.col(6) = 2520.0 * pow(t, 6) * pow(-t + 1, 2) - 10080.0 * pow(t, 5) * pow(-t + 1, 3) + 6300.0 * pow(t, 4) * pow(-t + 1, 4);
    Pddot.col(7) = -360.0 * pow(t, 7) * (2 * t - 2) - 5040.0 * pow(t, 6) * pow(-t + 1, 2) + 5040.0 * pow(t, 5) * pow(-t + 1, 3);
    Pddot.col(8) = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2 * t - 2) + 2520.0 * pow(t, 6) * pow(-t + 1, 2);
    Pddot.col(9) = -180.0 * pow(t, 8) + 72 * pow(t, 7) * (-10.0 * t + 10.0);
    Pddot.col(10) = 90.0 * pow(t, 8);

    Pdddot.col(0) = -720.0 * pow(-t + 1, 7);
    Pdddot.col(1) = 2160 * pow(1 - t, 7) - 5040 * pow(1 - t, 6) * t;
    Pdddot.col(2) = -15120 * pow(1-t, 5) * pow(t, 2) + 15120 * pow(1 - t, 6) * t - 2160 * pow(1 - t, 7);
    Pdddot.col(3) = -720 * pow(t-1, 4) * (120 * pow(t, 3) - 108 * pow(t, 2) + 24 * t - 1);
    Pdddot.col(4) = 5040 * pow(t-1, 3) * t * (30 * pow(t, 3) - 36*pow(t, 2) + 12 * t - 1);
    Pdddot.col(5) = -15120 * pow(t-1, 2) * pow(t, 2) * (12 * pow(t, 3) - 18*pow(t, 2) + 8 * t - 1);
    Pdddot.col(6) = 5040 * (t - 1) * pow(t, 3) * (30 * pow(t, 3)-54 * pow(t, 2) + 30 * t - 5);
    Pdddot.col(7) = -720 * pow(t, 7) + 10080 * (1-t) * pow(t, 6) - 45360 * pow(1 - t, 2) * pow(t, 5) + 25200 * pow(1-t, 3) * pow(t, 4) - 2520 * pow(t, 6) * (2 * t - 2);
    Pdddot.col(8) = 2160 * pow(t, 7) - 5040 * (1 - t) * pow(t, 6) + 15120 * pow(1-t, 2) * pow(t, 5) + 5040 * pow(t, 6) * (2 * t - 2);
    Pdddot.col(9) = 504 * (10 - 10 * t) * pow(t, 6) - 2160 * pow(t, 7);
    Pdddot.col(10) = 720.0 * pow(t, 7);

    Pddddot.col(0) = 5040.0 * pow(-t +1, 6);
    Pddddot.col(1) = -4320 * pow(t - 1, 5) * (10 * t - 3)-7200 * pow(t - 1, 6);
    Pddddot.col(2) = 10800 * pow(t - 1, 4) * (15 * pow(t, 2) - 9 * t + 1) + 2160 * pow(t-1, 5) * (30 * t - 9);
    Pddddot.col(3) = -20160 * pow(t-1, 3) * (30 * pow(t, 3) - 36 * pow(t, 2) + 12 * t - 1);
    Pddddot.col(4) = 5040 * pow(t-1, 2) * (210 * pow(t, 4) - 336 * pow(t, 3) + 168 * pow(t, 2) - 28 * t + 1);
    Pddddot.col(5) = -30240 * (t - 1) * t * (42 * pow(t, 4) - 84 * pow(t, 3) + 56 * pow(t, 2) - 14 * t + 1);
    Pddddot.col(6) = 1058400 * pow(t, 6) - 2540160 * pow(t, 5) + 2116800 * pow(t, 4) - 705600 * pow(t, 3) + 75600 * pow(t, 2);
    Pddddot.col(7) = -604800 * pow(t, 6) + 1088640 * pow(t, 5) - 604800 * pow(t, 4) + 100800 * pow(t, 3);
    Pddddot.col(8) = 226800 * pow(t, 6) - 272160 * pow(t, 5) + 75600 * pow(t, 4);
    Pddddot.col(9) = 30240 * pow(t, 5) - 50400 * pow(t, 6);
    Pddddot.col(10) = 5040.0 * pow(t, 6);


    s.a = P;
    s.b = Pdot/l;
    s.c = Pddot/(l*l);
    s.d = Pdddot / (l * l * l);
    s.e = Pddddot / (l * l * l * l);
  return s;
}

/**
 * @brief construct parameters to compute Polynomial Matrix, calls to bernsteinCoeffOrder10
 * @param horizon_length horizon-steps.
 * @param t_plan planning time (in seconds)
 * @return struct five_var : bernstein Matrices from (order 0 to 4th order) 
 */
five_var computeBernstein(float horizon_length, float t_plan){
    AXf total_time = AXf(horizon_length);
    total_time.setLinSpaced(horizon_length,0.0,t_plan);

    five_var bernsteinMatrix;
    //bernsteinCoeffOrder10(float n, float t_min, float t_max, Eigen::ArrayXXf t_actual, int num)
    bernsteinMatrix = bernsteinCoeffOrder10(10.0,total_time(0),total_time(-1),total_time,horizon_length);
    return bernsteinMatrix;
}

AXXf block_diag(AXXf arr1, AXXf arr2){
    AXXf temp(arr1.rows()+arr2.rows(),arr1.cols()+arr2.cols());
    temp = 0.0;
    temp.topRows(arr1.rows()).leftCols(arr1.cols())= arr1;
    temp.bottomRows(arr2.rows()).rightCols(arr2.cols()) = arr2;
    return temp;
}


AXXf stack(AXXf arr1, AXXf arr2, char ch){
    // vertical stack
    if (ch=='v') {
    AXXf temp(arr1.rows()+arr2.rows(),arr1.cols());
    temp<<arr1, arr2;
    return temp;
    }
    else if( ch=='h'){
    AXXf temp(arr1.rows(),arr1.cols()+arr2.cols());
    temp<<arr1, arr2;
    return temp;
    }
    else{
    std::cout<<"ERROR";
    }
}

AXXf reshape(AXXf x, uint32_t num_r, uint32_t num_c){
    Eigen::Map<Eigen::ArrayXXf> rx(x.data(),num_r,num_c);
    return rx;
}
/**
 * @brief construct parameters to compute Polynomial Matrix, calls to bernsteinCoeffOrder10
 * @param horizon_length horizon-steps.
 * @param t_plan planning time (in seconds)
 * @return struct five_var : bernstein Matrices from (order 0 to 4th order) 
 */
five_var solveBerstein(float horizon_length, float t_plan){
    AXf total_time = AXf(horizon_length);
    total_time.setLinSpaced(horizon_length,0.0,t_plan);

    five_var bernsteinMatrix;
    //bernsteinCoeffOrder10(float n, float t_min, float t_max, Eigen::ArrayXXf t_actual, int num)
    bernsteinMatrix = bernsteinCoeffOrder10(10.0,total_time(0),total_time(-1),total_time,horizon_length);
    return bernsteinMatrix;
}
#endif