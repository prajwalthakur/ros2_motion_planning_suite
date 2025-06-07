#include "qp_mpc_planner/diff_drive_formulation.hpp"


void init_prob(PlannerParam& planner_param){
    // Eigen::ArrayXf horizon_length  =  Eigen::ArrayXf(planner_param.num_horizon_length);
    // Eigen::ArrayXf horizon_length_up = Eigen::ArrayXf(planner_param.num_horizon_length_up);
    // horizon_length.setLinSpaced(planner_param.num_horizon_length, 0.0, planner_param.horizon_time);
    // horizon_length_up.setLinSpaced(planner_param.num_horizon_length_up, 0.0, planner_param.horizon_time);

    five_var bern_pol = computeBernstein(planner_param.num_horizon_length,planner_param.horizon_time);
    planner_param.nvar = bern_pol.a.cols();
    planner_param.P = bern_pol.a;
    planner_param.Pdot = bern_pol.b;
    planner_param.Pddot = bern_pol.c;
    planner_param.Pdddot = bern_pol.d;
    planner_param.Pddddot = bern_pol.e;

    five_var bern_pol_up = computeBernstein(planner_param.num_horizon_length_up,planner_param.horizon_time);
    planner_param.P_up = bern_pol_up.a;
    planner_param.Pdot_up = bern_pol_up.b;
    planner_param.Pddot_up = bern_pol_up.c;
    planner_param.Pdddot_up = bern_pol_up.d;
    planner_param.Pddddot_up = bern_pol_up.e;

    planner_param.dt = planner_param.horizon_time / planner_param.num_horizon_length_up;

    planner_param.cost_goal = block_diag(planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.P.bottomRows(planner_param.kappa).matrix(),\
                                        planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.P.bottomRows(planner_param.kappa).matrix());

    planner_param.cost_smoothness = block_diag(planner_param.Pddot.transpose().matrix()*planner_param.Pddot.matrix(),\
                                               planner_param.Pddot.transpose().matrix()*planner_param.Pddot.matrix());

}

// create the relevant data for the neighbouring obstacles
void neighbouring_obs(PlannerParam& planner_param){
    int N = planner_param.num_horizon_length;
    planner_param.x_obs_filt = AXXf::Ones(planner_param.x_obs.rows(),N);
    planner_param.y_obs_filt = AXXf::Ones(planner_param.x_obs.rows(),N);
    
    AXXf val = pow(((-planner_param.x_obs).rowwise() +  planner_param.x_ego)/(2*planner_param.lx_veh + planner_param.prox_obs),2) +\
                pow(((-planner_param.y_obs).rowwise() +  planner_param.y_ego)/(2*planner_param.ly_veh + planner_param.prox_obs),2) ;
   
    int k=0;
    for(int i=0;i<planner_param.x_obs.rows();++i){
        if((val.row(i)<=1.0).any()){
            planner_param.x_obs_filt.row(k) = planner_param.x_obs.row(i).rightCols(N);
            planner_param.y_obs_filt.row(k) = planner_param.y_obs.row(i).rightCols(N);
            k++;
        }
    }
    if(k!=0){
        planner_param.x_obs_filt.conservativeResize(k,N);
        planner_param.y_obs_filt.conservativeResize(k,N);
        planner_param.a_veh = AXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.lx_veh + planner_param.buffer);
        planner_param.b_veh = AXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.ly_veh + planner_param.buffer);
    }else{
        planner_param.x_obs_filt.conservativeResize(k, N);
        planner_param.y_obs_filt.conservativeResize(k, N);
        planner_param.a_veh = AXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.lx_veh + planner_param.buffer);
        planner_param.b_veh = AXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.ly_veh + planner_param.buffer);
    }
    planner_param.num_filt_obs = k;
}

void create_prob(const Eigen::ArrayXXf& ref_poses, const Eigen::Array3Xf& ego_pose,const Eigen::ArrayXXf& obs_poses, PlannerParam& planner_param){
    int num_obs = obs_poses.rows();
    int num_horizon = planner_param.num_horizon_length;
    planner_param.x_ego.resize(1,num_horizon);
    planner_param.y_ego.resize(1,num_horizon);
    
    planner_param.x_ego = ego_pose.row(0);
    planner_param.y_ego = ego_pose.row(1);

    planner_param.x_init  = ego_pose(0,0);
    planner_param.y_init  = ego_pose(1,0);
    planner_param.vx_init = 0.0; //TODO:
    planner_param.vy_init = 0.0; //TODO:
    planner_param.ax_init = 0.0;
    planner_param.ay_init = 0.0;

    planner_param.x_ref = ref_poses.col(0);
    planner_param.y_ref =  ref_poses.col(1);

    planner_param.x_obs.resize(num_obs,num_horizon);
    planner_param.y_obs.resize(num_obs,num_horizon);
    for(int i=0;i<num_horizon;++i){
        int co = 3*i;
        planner_param.x_obs.col(i) =  obs_poses.col(co);
        planner_param.y_obs.col(i) =  obs_poses.col(co+1);
    }
    neighbouring_obs(planner_param);
}


void solve_prob(PlannerParam& planner_param){

    // Inequality constraints

    AXXf A_pos_ineq = block_diag(stack(planner_param.P,-planner_param.P,'v'),stack(planner_param.P,-planner_param.P,'v')); // Ax>=b ==> -Ax<=b
    AXXf A_vel_ineq = block_diag(stack(planner_param.Pdot,-planner_param.Pdot,'v'),stack(planner_param.Pdot,-planner_param.Pdot,'v')); // Ax>=b ==> -Ax<=b
    AXXf A_acc_ineq = block_diag(stack(planner_param.Pddot,-planner_param.Pddot,'v'),stack(planner_param.Pddot,-planner_param.Pddot,'v')); // Ax>=b ==> -Ax<=b

    // (b)
    AXXf b_x_ineq = stack(planner_param.x_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.x_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_y_ineq = stack(planner_param.y_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.y_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_pos_ineq = stack(b_x_ineq,b_y_ineq,'v');

    AXXf b_vx_ineq = stack(planner_param.vel_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.vel_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_vy_ineq = stack(planner_param.vel_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.vel_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_vel_ineq = stack(b_vx_ineq,b_vy_ineq,'v');    
    // (c)
    AXXf b_ax_ineq = stack(planner_param.acc_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.acc_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_ay_ineq = stack(planner_param.acc_max*AXXf::Ones(planner_param.num_horizon_length,1),planner_param.acc_min*AXXf::Ones(planner_param.num_horizon_length,1),'v'); 
    AXXf b_acc_ineq = stack(b_ax_ineq, b_ay_ineq, 'v');
    
    // equality constraints for initial conditions

    planner_param.A_eq = block_diag(stack(stack(planner_param.P.row(0),planner_param.Pdot.row(0),'v'),planner_param.Pddot.row(0),'v'),\
                                    stack(stack(planner_param.P.row(0),planner_param.Pdot.row(0),'v'),planner_param.Pddot.row(0),'v'));
    planner_param.b_eq = AXXf(6,1); //x,vx,ax,y,vy,ay

    planner_param.b_eq<<planner_param.x_init, planner_param.vx_init, planner_param.ax_init, planner_param.y_init , planner_param.vy_init, planner_param.ay_init;


    // linear part of the quadratic cost ()
    //TODO:
    planner_param.lincost_goal = -1.0f*stack(\
                                planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.x_ref.bottomRows(planner_param.kappa).matrix(),\
                                planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.y_ref.matrix(),'v');


    AXXf A_slack_ineq, b_slack_ineq;
    int tries = 0;
    planner_param.qp_fail = 1;
    planner_param.weight_lin_slack =  planner_param.weight_lin_slack_og;
    planner_param.weight_quad_slack = planner_param.weight_quad_slack_og;
    int n_horizon = planner_param.num_horizon_length;
    int n_obs = planner_param.num_filt_obs;
    while(tries<1 && planner_param.qp_fail){
        Eigen::ArrayXXf temp_cost, temp_lincost;
        if(planner_param.num_filt_obs !=0){
            if(tries==0){
                // create extra param for the axis wise obs-avoidance ineq
                continous_collision_avoidance(planner_param);
                planner_param.slack = AXXf::Zero(n_horizon*n_obs,n_horizon*n_obs);
                planner_param.A_eq = block_diag(planner_param.A_eq,AXXf::Zero(n_horizon*n_obs,n_horizon*n_obs));
                A_pos_ineq = block_diag(A_pos_ineq,AXXf::Zero(n_horizon*n_obs,n_horizon*n_obs));


            }
        }else{
            planner_param.A_ineq = stack(stack(A_pos_ineq,A_vel_ineq,'v'),A_acc_ineq,'v');
            planner_param.b_ineq = stack(stack(b_pos_ineq,b_vel_ineq,'v'), b_acc_ineq,'v');
            temp_cost =  planner_param.weight_quad_slack*planner_param.cost_goal + planner_param.weight_smoothness*planner_param.cost_smoothness;
            temp_lincost = planner_param.weight_goal*planner_param.lincost_goal;
        }
    }

    // solve qp

    Eigen::QuadProgDense solver_

}

void continous_collision_avoidance(PlannerParam& planner_param){



}

