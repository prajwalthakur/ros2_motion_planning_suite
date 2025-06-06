#ifndef QP_MPC_PLANNER_QP_MPC_CLASS_HPP_
#define QP_MPC_PLANNER_QP_MPC_CLASS_HPP_
#pragma once 

#include <iostream>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <project_utils/common_utils.hpp>

#include <qp_mpc_planner/get_obs_pose.hpp>
#include <chrono>
#include <unsupported/Eigen/Splines>
struct PathDef{
    Eigen::Spline<float, 3> cs_pose;
    float arc_length;
    Eigen::RowVectorXf arc_vec; 
    int num_points;
    Eigen::Spline<float, 1> cs_x;
    Eigen::ArrayXXf ref_poses;
    //Eigen::Matrix<float,3,-1> ref_pose;
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
        float m_dist_threshold = 4.0 ;
    public:
        void ref_wp_section(int , int, const Eigen::ArrayXXf & ,Eigen::ArrayXXf& );
        Eigen::Index find_closest_point(MapArrayXfRow& ,Eigen::Array3f&);
        void find_ref_path( StateVector& );
        PathDef ref_wp_spline(const Eigen::ArrayXXf&);
        AXXf stack(const AXXf & , const AXXf &, char );
    private:
        int path_num_points = 30;

};  

#endif