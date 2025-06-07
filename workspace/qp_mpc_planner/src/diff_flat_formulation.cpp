#include "qp_mpc_planner/diff_flat_formulation.hpp"

namespace diffflatformulation{
    void init_prob(PlannerParam& planner_param){
        // Eigen::ArrayXf horizon_length  =  Eigen::ArrayXf(planner_param.num_horizon_length);
        // Eigen::ArrayXf horizon_length_up = Eigen::ArrayXf(planner_param.num_horizon_length_up);
        // horizon_length.setLinSpaced(planner_param.num_horizon_length, 0.0, planner_param.horizon_time);
        // horizon_length_up.setLinSpaced(planner_param.num_horizon_length_up, 0.0, planner_param.horizon_time);

        bernsteinpol::five_var bern_pol = bernsteinpol::computeBernstein(planner_param.num_horizon_length,planner_param.horizon_time);
        planner_param.nvar = bern_pol.a.cols();
        planner_param.P = bern_pol.a;
        planner_param.Pdot = bern_pol.b;
        planner_param.Pddot = bern_pol.c;
        planner_param.Pdddot = bern_pol.d;
        planner_param.Pddddot = bern_pol.e;

        bernsteinpol::five_var bern_pol_up = bernsteinpol::computeBernstein(planner_param.num_horizon_length_up,planner_param.horizon_time);
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
        planner_param.x_obs_filt = Eigen::ArrayXXf::Ones(planner_param.x_obs.rows(),N);
        planner_param.y_obs_filt = Eigen::ArrayXXf::Ones(planner_param.x_obs.rows(),N);
        
        Eigen::ArrayXXf val = pow(((-planner_param.x_obs).rowwise() +  planner_param.x_ego)/(2*planner_param.lx_veh + planner_param.prox_obs),2) +\
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
            planner_param.a_veh = Eigen::ArrayXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.lx_veh + planner_param.buffer);
            planner_param.b_veh = Eigen::ArrayXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.ly_veh + planner_param.buffer);
        }else{
            planner_param.x_obs_filt.conservativeResize(k, N);
            planner_param.y_obs_filt.conservativeResize(k, N);
            planner_param.a_veh = Eigen::ArrayXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.lx_veh + planner_param.buffer);
            planner_param.b_veh = Eigen::ArrayXXf::Ones(planner_param.x_obs_filt.rows(),N)*(2*planner_param.ly_veh + planner_param.buffer);
        }
        planner_param.num_filt_obs = k;
    }
    //pred_obs_poses num_obsx(3*horizon_length)
    //pred_ego_pose 3x(horizon_length)
    //ref_poses 3x(horizon_length)
    // planner_param PlannerParam class stores paramter and computed values
    void create_prob(const Eigen::ArrayXXf& ref_poses, const Eigen::Array3Xf& pred_ego_pose,const Eigen::ArrayXXf& pred_obs_poses, PlannerParam& planner_param){
        int num_obs = pred_obs_poses.rows();
        int num_horizon = planner_param.num_horizon_length;
        planner_param.x_ego.resize(1,num_horizon);
        planner_param.y_ego.resize(1,num_horizon);
        
        planner_param.x_ego = pred_ego_pose.row(0);
        planner_param.y_ego = pred_ego_pose.row(1);

        planner_param.x_init  = pred_ego_pose(0,0);
        planner_param.y_init  = pred_ego_pose(1,0);
        planner_param.vx_init = 0.0; //TODO:
        planner_param.vy_init = 0.0; //TODO:
        planner_param.ax_init = 0.0;
        planner_param.ay_init = 0.0;

        planner_param.x_ref = ref_poses.row(0);
        planner_param.y_ref =  ref_poses.row(1);

        planner_param.x_obs.resize(num_obs,num_horizon);
        planner_param.y_obs.resize(num_obs,num_horizon);
        for(int i=0;i<num_horizon;++i){
            int co = 3*i;
            planner_param.x_obs.col(i) =  pred_obs_poses.col(co);
            planner_param.y_obs.col(i) =  pred_obs_poses.col(co+1);
        }
        neighbouring_obs(planner_param);
    }


    void solve_prob(PlannerParam& planner_param){

        // Inequality constraints

        Eigen::ArrayXXf A_pos_ineq = block_diag(stack(planner_param.P,-planner_param.P,'v'),stack(planner_param.P,-planner_param.P,'v')); // Ax>=b ==> -Ax<=b
        Eigen::ArrayXXf A_vel_ineq = block_diag(stack(planner_param.Pdot,-planner_param.Pdot,'v'),stack(planner_param.Pdot,-planner_param.Pdot,'v')); // Ax>=b ==> -Ax<=b
        Eigen::ArrayXXf A_acc_ineq = block_diag(stack(planner_param.Pddot,-planner_param.Pddot,'v'),stack(planner_param.Pddot,-planner_param.Pddot,'v')); // Ax>=b ==> -Ax<=b

        // (b)
        Eigen::ArrayXXf b_x_ineq = stack(planner_param.x_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.x_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_y_ineq = stack(planner_param.y_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.y_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_pos_ineq = stack(b_x_ineq,b_y_ineq,'v');

        Eigen::ArrayXXf b_vx_ineq = stack(planner_param.vel_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.vel_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_vy_ineq = stack(planner_param.vel_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.vel_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_vel_ineq = stack(b_vx_ineq,b_vy_ineq,'v');    
        // (c)
        Eigen::ArrayXXf b_ax_ineq = stack(planner_param.acc_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.acc_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_ay_ineq = stack(planner_param.acc_max*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),planner_param.acc_min*Eigen::ArrayXXf::Ones(planner_param.num_horizon_length,1),'v'); 
        Eigen::ArrayXXf b_acc_ineq = stack(b_ax_ineq, b_ay_ineq, 'v');
        
        // equality constraints for initial conditions

        planner_param.A_eq = block_diag(stack(stack(planner_param.P.row(0),planner_param.Pdot.row(0),'v'),planner_param.Pddot.row(0),'v'),\
                                        stack(stack(planner_param.P.row(0),planner_param.Pdot.row(0),'v'),planner_param.Pddot.row(0),'v'));
        planner_param.b_eq = Eigen::ArrayXXf(6,1); //x,vx,ax,y,vy,ay

        planner_param.b_eq<<planner_param.x_init, planner_param.vx_init, planner_param.ax_init, planner_param.y_init , planner_param.vy_init, planner_param.ay_init;


        // linear part of the quadratic cost ()
        //TODO:
        planner_param.lincost_goal = -1.0f*stack(\
                                    planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.x_ref.transpose().bottomRows(planner_param.kappa).matrix(),\
                                    planner_param.P.bottomRows(planner_param.kappa).transpose().matrix()*planner_param.y_ref.transpose().bottomRows(planner_param.kappa).matrix(),'v');


        Eigen::ArrayXXf A_slack_ineq, b_slack_ineq;
        int tries = 0;
        planner_param.qp_fail = 0;
        planner_param.weight_lin_slack =  planner_param.weight_lin_slack_og;
        planner_param.weight_quad_slack = planner_param.weight_quad_slack_og;
        int n_horizon = planner_param.num_horizon_length;
        int n_obs = planner_param.num_filt_obs;
        while(tries<1 && (!planner_param.qp_fail)){
            Eigen::ArrayXXf temp_cost, temp_lincost;
            if(planner_param.num_filt_obs !=0){
                if(tries==0){
                    // create extra param for the axis wise obs-avoidance ineq
                    continous_collision_avoidance(planner_param);
                    planner_param.slack = Eigen::ArrayXXf::Zero(n_horizon*n_obs,n_horizon*n_obs);
                    planner_param.A_eq = block_diag(planner_param.A_eq,Eigen::ArrayXXf::Zero(n_horizon*n_obs,n_horizon*n_obs));
                    A_pos_ineq = block_diag(A_pos_ineq,Eigen::ArrayXXf::Zero(n_horizon*n_obs,n_horizon*n_obs));


                }
            }else{
                planner_param.A_ineq = stack(stack(A_pos_ineq,A_vel_ineq,'v'),A_acc_ineq,'v');
                planner_param.b_ineq = stack(stack(b_pos_ineq,b_vel_ineq,'v'), b_acc_ineq,'v');
                temp_cost =  planner_param.weight_quad_slack*planner_param.cost_goal + planner_param.weight_smoothness*planner_param.cost_smoothness;
                temp_lincost = planner_param.weight_goal*planner_param.lincost_goal;
            }
        
        // solve qp

        Eigen::QuadProgDense solver_xy(temp_cost.rows(),planner_param.A_eq.rows(), planner_param.A_ineq.rows());   
        

        solver_xy.solve((temp_cost).cast<double>().matrix(),temp_lincost.cast<double>().matrix(),\
                            planner_param.A_eq.cast<double>().matrix(), planner_param.b_eq.cast<double>().matrix(),\
                            planner_param.A_ineq.cast<double>().matrix(), planner_param.b_ineq.cast<double>().matrix());

        Eigen::ArrayXXf sol = solver_xy.result().cast<float>();
        Eigen::ArrayXXf sol_xy  = sol.topRows(2*planner_param.nvar);
        Eigen::ArrayXXf sol_x = sol_xy.topRows(planner_param.nvar);
        Eigen::ArrayXXf sol_y = sol_xy.bottomRows(planner_param.nvar);

        if(planner_param.num_filt_obs!=0){

            planner_param.slack = sol.bottomRows(planner_param.num_filt_obs*planner_param.num_horizon_length);
        }

        planner_param.x = planner_param.P.matrix()*sol_x.matrix();
        planner_param.y = planner_param.P.matrix()*sol_y.matrix();

        planner_param.xdot = planner_param.Pdot.matrix()*sol_x.matrix();
        planner_param.ydot = planner_param.Pdot.matrix()*sol_y.matrix();

        planner_param.xddot = planner_param.Pddot.matrix()*sol_x.matrix();
        planner_param.yddot = planner_param.Pddot.matrix()*sol_x.matrix();

        
        planner_param.x_up = planner_param.P_up.matrix()*sol_x.matrix();
        planner_param.y_up = planner_param.P_up.matrix()*sol_y.matrix();

        planner_param.xdot_up = planner_param.Pdot_up.matrix()*sol_x.matrix();
        planner_param.ydot_up = planner_param.Pdot_up.matrix()*sol_y.matrix();

        planner_param.xddot_up = planner_param.Pddot_up.matrix()*sol_x.matrix();
        planner_param.yddot_up = planner_param.Pddot_up.matrix()*sol_x.matrix();    

        planner_param.qp_fail = solver_xy.fail();
        tries++;
        }
    }

    void continous_collision_avoidance(PlannerParam& planner_param){


    }

    void compute_controls(PlannerParam& planner_param){
        if(planner_param.qp_fail)
        {   
            // if planner fails to compute solution
            planner_param.x =    stack(planner_param.x.bottomRows(planner_param.x.rows()-1) ,planner_param.x.bottomRows(1), 'v');
            planner_param.y =    stack(planner_param.y.bottomRows(planner_param.y.rows()-1) ,planner_param.y.bottomRows(1), 'v');
            planner_param.xdot = stack(planner_param.y.bottomRows(planner_param.xdot.rows()-1) ,planner_param.xdot.bottomRows(1), 'v');
            planner_param.ydot = stack(planner_param.ydot.bottomRows(planner_param.ydot.rows()-1) ,planner_param.ydot.bottomRows(1), 'v');

            planner_param.xddot = stack(planner_param.y.bottomRows(planner_param.xddot.rows()-1) ,planner_param.xddot.bottomRows(1), 'v');
            planner_param.yddot = stack(planner_param.yddot.bottomRows(planner_param.yddot.rows()-1) ,planner_param.yddot.bottomRows(1), 'v');

            
            planner_param.x_up = stack(planner_param.x_up.bottomRows(planner_param.x_up.rows()-1) ,planner_param.x_up.bottomRows(1), 'v');
            planner_param.y_up = stack(planner_param.y_up.bottomRows(planner_param.y_up.rows()-1) ,planner_param.y_up.bottomRows(1), 'v');

            planner_param.xdot_up = stack(planner_param.xdot_up.bottomRows(planner_param.xdot_up.rows()-1) ,planner_param.xdot_up.bottomRows(1), 'v');
            planner_param.ydot_up = stack(planner_param.ydot_up.bottomRows(planner_param.ydot_up.rows()-1) ,planner_param.ydot_up.bottomRows(1), 'v');

            planner_param.xddot_up = stack(planner_param.xddot_up.bottomRows(planner_param.xddot_up.rows()-1) ,planner_param.xddot_up.bottomRows(1), 'v');
            planner_param.yddot_up = stack(planner_param.yddot_up.bottomRows(planner_param.yddot_up.rows()-1) ,planner_param.yddot_up.bottomRows(1), 'v');
        }
        Eigen::ArrayXXf commanded_speed  =    (planner_param.xdot*planner_param.xdot + planner_param.ydot*planner_param.ydot).rowwise().norm();

        Eigen::ArrayXXf curvature       =    ((planner_param.yddot*planner_param.xdot - planner_param.ydot*planner_param.xddot) \
                                        /pow((planner_param.xdot*planner_param.xdot + planner_param.ydot*planner_param.ydot),1.5));

        Eigen::ArrayXXf steer_angle     = planner_param.wheel_base*curvature.atan() ; 

        Eigen::ArrayXf commanded_yaw  =  planner_param.ydot.binaryExpr(planner_param.xdot, [](float y, float x) {
                                return std::atan2(y, x);
                            });

        for(size_t i=1;i<commanded_yaw.rows()-1;++i){
            float delta = commanded_yaw(i) - commanded_yaw(i-1);
            if(delta > M_PI){commanded_yaw(i) -= 2.0f*M_PI; }
            else if(delta < -M_PI){ commanded_yaw(i) += 2.0f*M_PI; }
        }

        planner_param.commanded_speed = commanded_speed;
        planner_param.commanded_curvature = curvature;
        planner_param.commanded_steer_angle  = steer_angle;
        planner_param.commanded_yaw = commanded_yaw;
    }



    Eigen::ArrayXXf stack(const Eigen::ArrayXXf & arr1, const Eigen::ArrayXXf & arr2, char ch){
        // vertical stack
        if (ch=='v') {
        Eigen::ArrayXXf temp(arr1.rows()+arr2.rows(),arr1.cols());
        temp.topRows(arr1.rows()) =  arr1;
        temp.bottomRows(arr2.rows()) = arr2;
        return temp;
        }
        else if( ch=='h'){
        Eigen::ArrayXXf temp(arr1.rows(),arr1.cols()+arr2.cols());
        //RCLCPP_INFO_STREAM(this->get_logger(), arr1.rows()<<" " << arr1.cols()<<" " << arr2.cols());

        temp.leftCols(arr1.cols()) = arr1;
        temp.rightCols(arr2.cols()) = arr2;
        return temp;
        }
        else{
        return Eigen::ArrayXXf();
        }

    }

    Eigen::ArrayXXf block_diag( const Eigen::ArrayXXf & arr1, const Eigen::ArrayXXf & arr2){
        Eigen::ArrayXXf temp(arr1.rows()+arr2.rows(),arr1.cols()+arr2.cols());
        temp = 0.0;
        temp.topRows(arr1.rows()).leftCols(arr1.cols())= arr1;
        temp.bottomRows(arr2.rows()).rightCols(arr2.cols()) = arr2;
        return temp;
    }
}