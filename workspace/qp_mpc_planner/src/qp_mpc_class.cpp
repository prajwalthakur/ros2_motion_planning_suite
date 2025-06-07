#include "qp_mpc_planner/qp_mpc_class.hpp"
QpMpc::QpMpc():Node("qp_mpc_planner_node") {
    RCLCPP_INFO(this->get_logger(),"qp mpc node started");
    std::string map_path = "/root/workspace/src/project_utils/maps/e7_floor5_square.csv";
    load_map(map_path,path_data_points,mat_path_points);
    RCLCPP_INFO(this->get_logger(),"load map successfully called");
    RCLCPP_INFO_STREAM(this->get_logger(), "Rows = " <<  mat_path_points.rows() << ", Cols = " << mat_path_points.cols());
    Eigen::VectorXd ego_state(5);
    ego_state<<10.0,0.0,0.0,0.0,0.0;

    // initialize the problem formulation of diff flatness base
    diffflatformulation::init_prob(planner_param);
    
    find_ref_path( ego_state);
    //find_closest_point(mat_path_points,ego_state);
    RCLCPP_INFO(this->get_logger(),"find_ref_path successfully called");


}

Eigen::Index QpMpc::find_closest_point(MapArrayXfRow& path_array, Eigen::Array3f& ego_state){
    
    // get the position of the ego vehicle
    AXf current_ego_pos(2);
    current_ego_pos<<ego_state(0),ego_state(1);
    //RCLCPP_INFO(this->get_logger(),"inside find_closest_point ");
    // find the closest pose to the path_array from ego_vehicle
    int R = static_cast<int>(path_array.rows());
    auto xy_block = path_array.block(0, 0, R, 2); // R×2
    Eigen::ArrayXf diffs = (xy_block.rowwise() - current_ego_pos.transpose().row(0)).square().rowwise().sum();
    Eigen::Index idx;
    diffs.minCoeff(&idx);
    //RCLCPP_INFO_STREAM(this->get_logger(), "Rows = " <<  diffs.rows() << ", Cols = " << diffs.cols());
    Eigen::Vector2f closest_pt;
    closest_pt << path_array(idx, 0), path_array(idx, 1);
    //RCLCPP_INFO_STREAM(this->get_logger(), "closest_pt = " <<  closest_pt(0));
    return idx;
}


AXXf QpMpc::stack(const AXXf & arr1, const AXXf & arr2, char ch){
  // vertical stack
  if (ch=='v') {
    AXXf temp(arr1.rows()+arr2.rows(),arr1.cols());
    temp.topRows(arr1.rows()) =  arr1;
    temp.bottomRows(arr2.rows()) = arr2;
    return temp;
  }
  else if( ch=='h'){
    AXXf temp(arr1.rows(),arr1.cols()+arr2.cols());
    //RCLCPP_INFO_STREAM(this->get_logger(), arr1.rows()<<" " << arr1.cols()<<" " << arr2.cols());
    
    temp.leftCols(arr1.cols()) = arr1;
    temp.rightCols(arr2.cols()) = arr2;
    return temp;
  }
  else{
    return AXXf();
  }

}

PathDef QpMpc::ref_wp_spline(const  Eigen::ArrayXXf& ref_wp){
    //RCLCPP_INFO_STREAM(this->get_logger(), "in ref_wp_spline");
    size_t N = static_cast<size_t>(ref_wp.rows());
    Eigen::ArrayXXf X_diff = ref_wp.bottomRows(N-1) - ref_wp.topRows(N-1);
    Eigen::ArrayXf phi(N);
    Eigen::ArrayXf dx = X_diff.col(0); // size (N-1)
    Eigen::ArrayXf dy = X_diff.col(1); // size (N-1)
    phi = dy.binaryExpr(dx, [](float y, float x) {
      return std::atan2(y, x);
    });
    for(size_t i=1;i<N-1;++i){
        float delta = phi(i) - phi(i-1);
        if(delta > M_PI){phi(i) -= 2.0f*M_PI; }
        else if(delta < -M_PI){ phi(i) += 2.0f*M_PI; }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), phi.rows()<<" "<<N);
    //phi(N-1) = phi(N-2);
    Eigen::ArrayXf phi_full(N);
    phi_full.head(N - 1) = phi;  
    phi_full(N-1) = phi(N-2);  
    //Eigen::ArrayXXf exam = {{1,1},{2,2},{3,3}};       
    //Eigen::ArrayXf seg_len = ((exam.square().rowwise().sum()).sqrt()).sum(); //(X_diff).rowwise().squaredNorm();
    float arc_length =  (X_diff).rowwise().norm().sum();  //((X_diff.square().rowwise().sum()).sqrt()).sum(); //seg_len.sum();
    //RCLCPP_INFO_STREAM(this->get_logger(), arc_length << "--" << (X_diff).rowwise().norm().sum());
    Eigen::ArrayXf arc_vec(N);
    arc_vec.setLinSpaced(N,0.0,arc_length); 
    Eigen::ArrayXXf phi_col = phi_full.matrix();  // now (N)×1
    //phi_col.resize(N , 1);  
    Eigen::ArrayXXf X_pose = stack(ref_wp,phi_col,'h');
    //RCLCPP_INFO_STREAM(this->get_logger(), "before transpose");
    Eigen::Matrix3Xf X_row_pose = X_pose.transpose(); //3X(N-1)
    const int spline_degree = 3;
    using Spline3d =  Eigen::Spline<float,3>;
    // interpolate X_pose()
    //RCLCPP_INFO_STREAM(this->get_logger(), "arc_vec"<< arc_vec(0) << arc_vec(1) << arc_vec(2)<<"---arc-length"<<arc_length<<arc_vec.size());
    Spline3d  cs_pose = Eigen::SplineFitting<Spline3d>::Interpolate(
        X_row_pose,
        spline_degree,
        arc_vec );
    
    Eigen::ArrayXXf interep_poses(N,3);
    for(size_t i =0 ; i< N;++i){
        float s = arc_vec(i);
        interep_poses.row(i) = cs_pose(s).array().transpose();
    }


    PathDef path_def;
    path_def.arc_length = arc_length;
    path_def.arc_vec = arc_vec;
    path_def.cs_pose = std::move(cs_pose);
    path_def.num_points = N;
    path_def.ref_poses = std::move(interep_poses);
    //RCLCPP_INFO_STREAM(this->get_logger(), cs_pose(arc_length));
    return path_def;
}

/**
 * @brief Extracts a continuous segment of reference waypoints starting from the closest point.
 *
 * This function returns a fixed-length segment of the reference path for local planning,
 * beginning at the index of the waypoint closest to the current position. If the segment 
 * extends beyond the end of the path, it wraps around to the beginning.
 *
 * @param idx_int          Index of the closest waypoint on the reference path.
 * @param path_num_points  Number of waypoints to extract starting from idx_int.
 * @param mat_path_points  Full list of reference waypoints (Nx2 array).
 * @param ref_wp           Output array to store the extracted segment of waypoints (path_num_points x 2).
 */
void QpMpc::ref_wp_section(int idx_int, int path_num_points, const Eigen::ArrayXXf & mat_path_points, Eigen::ArrayXXf& ref_wp) {
    int total_rows = static_cast<int>(mat_path_points.rows());
    int n = std::min(total_rows, path_num_points); 
    if (n + idx_int <= total_rows) {
        // Extract a continuous block if it fits within bounds
        ref_wp = mat_path_points.block(idx_int, 0, path_num_points, 2);
    } else {
        // Wrap around: extract from idx_int to end, then from start
        int n1 = total_rows - idx_int;
        int n2 = n - n1;
        ref_wp.topRows(n1) = mat_path_points.block(idx_int, 0, n1, 2);
        ref_wp.bottomRows(n2) = mat_path_points.block(0, 0, n2, 2);
    }
}


void QpMpc::find_ref_path( StateVector& ego_state){
    Eigen::Array3f ego_pose;
    ego_pose<<ego_state(0),ego_state(1),ego_state(2);
    Eigen::Index idx = find_closest_point(mat_path_points, ego_pose);
    int idx_int  = static_cast<int>(idx);
    int total_rows = static_cast<int>(mat_path_points.rows());
    int n = std::min(total_rows,path_num_points);
    // Prepare an output array of size (n × 2)
    Eigen::ArrayXXf ref_wp(n, 2);
    ref_wp_section(idx_int, path_num_points, mat_path_points,ref_wp);
    PathDef path_def =   ref_wp_spline(ref_wp);
    Eigen::ArrayXXf obs_poses = extract_near_by_obs(ego_pose,m_dist_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(), "obs-poses" <<obs_poses);

    size_t obs_len = obs_poses.rows();


    diffflatformulation::create_prob(ref_poses, ego_pose,obs_poses,planner_param);
    diffflatformulation::create_prob(ref_poses, ego_pose,obs_poses,planner_param);
    diffflatformulation::solve_prob(planner_param);
    RCLCPP_INFO_STREAM(this->get_logger(), "Qp solved sucessfully?" <<planner_param.qp_fail);
    diffflatformulation::compute_controls(planner_param);
    RCLCPP_INFO_STREAM(this->get_logger(), "compute_controls called and successfully implemented");
    //Eigen::ArrayXXf ref_pose(path_def.num_points,3); //x,y,yaw
    // Eigen::Matrix<float,3,1> XYZPhi = path_def.cs_pose(path_def.arc_length);
    // float x_s   = XYZPhi(0);
    // float y_s   = XYZPhi(1);
    // float phi_s = XYZPhi(2);
    // float arc_length = path_def.arc_length;
    // RCLCPP_INFO_STREAM(this->get_logger(), "poses" <<path_def.ref_poses << "x_s = " <<  x_s << ", y_s = " << y_s<< "phi_s "<< phi_s<< "arc-length"<<arc_length);

}
