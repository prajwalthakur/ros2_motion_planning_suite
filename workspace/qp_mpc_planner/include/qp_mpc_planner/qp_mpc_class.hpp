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
#include <unsupported/Eigen/Splines>
struct PathDef{

    AXf cs_x_path,cs_y_path,cs_phi_path,arc_length, arc_vec;
};


class QpMpc: public rclcpp::Node{
    public:
        QpMpc();
        //void on_activate();
    private:
        std::vector<float> path_data_points;
        MapArrayXfRow mat_path_points{nullptr, 0, 0};  // “placeholder” Map
        std::shared_ptr<VehicleClass> m_vehicle;
        rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr m_control_publisher;
        rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr m_state_subscriber;
        InputVector m_control_ref;
        rclcpp::TimerBase::SharedPtr m_control_pub_timer;
    public:
        Eigen::Index find_closest_point(MapArrayXfRow& ,StateVector&);
        void find_ref_path( StateVector& );
        PathDef ref_wp_spline(Eigen::ArrayXXf&);
    private:
        int path_num_points = 100;

};  

#endif