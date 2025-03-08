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
                std::function<void(const StateVector&)> setStates,
                std::function<void(const InputVector&)> setInput) 
            : dynamics_(dynamics) , setStates_(setStates) , setInput_(setInput){}


        StateVector rk4Integrator(const StateVector& , const InputVector& ,double )const;
        StateVector efIntegrator(const StateVector& , const InputVector& ,double )const;
        void simNextState(const StateVector& , const InputVector& ,double )const;
    private:
        const double fine_time_step_ = 0.001;      
        std::function<StateVector(const StateVector&, const InputVector&)> dynamics_;  
        std::function<void(const StateVector&)> setStates_;
        std::function<void(const InputVector&)> setInput_;

};
