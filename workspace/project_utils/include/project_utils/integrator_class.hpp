#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <functional>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector;

class IntegratorClass{
    public:
        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics) 
            : dynamics_(dynamics) {}

        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
                std::function<const StateVector&()> getState,
                std::function<void(const StateVector&)> setState,
                std::function<void(const InputVector&)> setInput) 
            : dynamics_(dynamics) , setState_(setStates) , setInput_(setInput), getState_(getState){}


        StateVector rk4Integrator(const StateVector& , const InputVector& ,double )const;
        StateVector efIntegrator(const StateVector& , const InputVector& ,double )const;
        void simNextState(const InputVector& ,double )const;
    private:
        const double fine_time_step_ = 0.001;      
        std::function<StateVector(const StateVector&, const InputVector&)> dynamics_;  
        std::function<void(const StateVector&)> setState_;
        std::function<void(const InputVector&)> setInput_;
        std::function<const StateVector&()> getState_;
};
