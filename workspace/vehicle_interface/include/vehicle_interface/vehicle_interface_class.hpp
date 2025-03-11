#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/integrator_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
using namespace std::chrono_literals;

class VehicleInterfaceNode: public rclcpp::Node, public std::enable_shared_from_this<VehicleInterfaceNode>{
    public:
        VehicleInterfaceNode();
    private: 
        std::shared_ptr<VehicleClass> vehicle_;
        std::shared_ptr<IntegratorClass> integrator_;
        rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        project_utils::msg::EigenVector control_ref;
        void state_pub_timer_callback(); 
        void state_update_timer_callback();
        void control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg);
}