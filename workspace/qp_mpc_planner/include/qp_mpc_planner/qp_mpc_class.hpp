#ifndef QP_MPC_PLANNER_QP_MPC_CLASS_HPP_
#define QP_MPC_PLANNER_QP_MPC_CLASS_HPP_
#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <project_utils/common_utils.hpp>
#include <chrono>

class QpMpc: public rclcpp::Node{
    public:
        QpMpc();
        //void on_activate();
    private:
        std::vector<float> path_data_points;
        MapMatrixfRow mat_path_points{nullptr, 0, 0};  // “placeholder” Map
        std::shared_ptr<VehicleClass> m_vehicle;
        rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr m_control_publisher;
        rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr m_state_subscriber;
        InputVector m_control_ref;
        rclcpp::TimerBase::SharedPtr m_control_pub_timer;
};

#endif