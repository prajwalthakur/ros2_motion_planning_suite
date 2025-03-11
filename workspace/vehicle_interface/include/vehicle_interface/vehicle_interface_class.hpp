#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/integrator_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
using namespace std::chrono_literals;

class VehicleInterface: public rclcpp::Node{
    public:
        VehicleInterface();
        void on_activate();
    private: 
        std::shared_ptr<VehicleClass> vehicle_;
        std::shared_ptr<IntegratorClass> integrator_;
        rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr state_publisher_;
        rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr control_subscriber_;
        rclcpp::TimerBase::SharedPtr state_update_timer_;
        rclcpp::TimerBase::SharedPtr state_pub_timer_;
        InputVector control_ref;
        void state_pub_timer_callback(); 
        void state_update_timer_callback();
        void control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg);
        int NX;
        int NU;
        double integration_deltaT;
        double sim_deltaT;
        double state_publish_deltaT;
};