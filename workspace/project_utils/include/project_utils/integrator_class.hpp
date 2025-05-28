
#ifndef PROJECT_UTILS_INTEGRATOR_CLASS_HPP_
#define PROJECT_UTILS_INTEGRATOR_CLASS_HPP_
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <functional>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector; //the size is determined at runtime rather than compile time.


class IntegratorClass{
    public:
        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
                std::function<const StateVector&()> getState,
                std::function<void(const StateVector&)> setState,
                std::function<void(const InputVector&)> setInput,double fine_time_step) 
            : dynamics_(dynamics) , setState_(setState) , setInput_(setInput), getState_(getState), fine_time_step_(fine_time_step){}


        StateVector rk4Integrator(const StateVector& , const InputVector& ,double )const;
        StateVector efIntegrator(const StateVector& , const InputVector& ,double )const;
        void simNextState(const InputVector& ,double )const;
    private:
        const double fine_time_step_;      
        std::function<StateVector(const StateVector&, const InputVector&)> dynamics_;  
        std::function<void(const StateVector&)> setState_;
        std::function<void(const InputVector&)> setInput_;
        std::function<const StateVector&()> getState_;
};
#endif