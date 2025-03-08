#ifndef VEHCICLE_DYNAMICS_HPP_
#define VEHCICLE_DYNAMICS_HPP_

#include "rclcpp/rclcpp.hpp"

class VehicleDynamics.hpp : public rclcpp::Node{

    public:
        VehicleDynamics();
    private:
        void timer_callback_fwd_dynamics();
}