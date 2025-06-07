#ifndef QP_MPC_PLANNER_UTILS__HPP_
#define QP_MPC_PLANNER_UTILS__HPP_
#pragma once
struct PlannerParam{
    float num_horizon_length = 30.0;
    float num_horizon_length_up = 300.0;
    float horizon_time = 10.0;
    float vel_max = 5.0,vel_min = 0.0;
    float steer_max;
    float acc_max=2.0, acc_min = -2.0;
    float steer_dot_max;
    float wheel_base = 2.0;
    float lx_veh = 2.0, ly_veh = 0.5;
    float prox_obs=0.2, prox_agent=0.2;
    int kappa = num_horizon_length; 
    int num_filt_obs;
    Eigen::Array<float,1,-1> x_ego,y_ego;
    Eigen::ArrayXXf x_obs,y_obs,x_obs_filt,y_obs_filt;

    Eigen::ArrayXXf commanded_speed, curvature, steer_angle;

    
    bool mpc, free_space, on_demand;
    int num, num_up, num_ctrl, nvar,  VERBOSE, mpc_step, max_time, num_static_obs, num_veh, qp_fail, id_badge, colliding_step, world;

    float dt, t_plan, weight_smoothness, weight_goal, weight_quad_slack, weight_lin_slack, dist_to_goal, dist_stop, buffer, weight_lin_slack_og, weight_quad_slack_og;
    //float vel_max,vel_min, acc_max, acc_min, prox_obs, prox_agent, lx_veh, ly_veh, rmin;

    float x_min, y_min, z_min,
            x_max, y_max, z_max,
            x_init, y_init, z_init,
            x_goal, y_goal, z_goal, 
            vx_init, vy_init, vz_init,
            ax_init, ay_init, az_init;

    float gravity, f_min, f_max;
    float mean, stdev;
    bool use_thrust_values, use_model;

    AXXf A_coll, b_coll;
    AXXf agents_x, agents_y, agents_z; 
    AXXf A_eq, A_ineq, A_obs, cost_goal, cost_smoothness, cost_slack;
    AXXf b_eq, b_ineq, lincost_goal;

    AXXf P, Pdot, Pddot, Pdddot, Pddddot;  
    AXXf P_up, Pdot_up, Pddot_up, Pdddot_up, Pddddot_up;

    AXXf x_ref, y_ref, z_ref;
    AXXf slack;

    AXXf x_static_obs, y_static_obs, z_static_obs;
    AXXf x_static_obs_og, y_static_obs_og, z_static_obs_og;
    AXXf x_veh, y_veh, z_veh;

    AXXf a_veh, b_veh, c_veh;
    AXXf a_static_obs, b_static_obs, c_static_obs;
    AXXf a_static_obs_og, b_static_obs_og, c_static_obs_og; 

    AXXf x, y, z,
        xdot, ydot, zdot,
        xddot, yddot, zddot,
        xdddot, ydddot, zdddot,
        xddddot, yddddot, zddddot;
    
    AXXf x_up, y_up, z_up,
        xdot_up, ydot_up, zdot_up,
        xddot_up, yddot_up, zddot_up;
    
    std :: vector<float> smoothness, arc_length, inter_agent_dist, agent_obs_dist, inter_agent_dist_min, agent_obs_dist_min;
    std :: string solver;


    std :: vector<std :: vector<float>> pos_static_obs, dim_static_obs;


};
#endif



