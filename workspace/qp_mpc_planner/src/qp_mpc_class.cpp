#include "qp_mpc_planner/qp_mpc_class.hpp"
QpMpc::QpMpc():Node("qp_mpc_planner_node") {
    RCLCPP_INFO(this->get_logger(),"qp mpc node started");
    std::string map_path = "/root/workspace/src/project_utils/maps/e7_floor5_square.csv";
    load_map(map_path,path_data_points,mat_path_points);
    RCLCPP_INFO(this->get_logger(),"load map successfully called");
    RCLCPP_INFO_STREAM(this->get_logger(), "Rows = " <<  mat_path_points.rows() << ", Cols = " << mat_path_points.cols());
    Eigen::VectorXd ego_state(5);
    ego_state<<40.0,40.0,0.0,0.0,0.0;
    find_closest_point(mat_path_points,ego_state);
    RCLCPP_INFO(this->get_logger(),"find_closest_point successfully called");


}

Eigen::Index QpMpc::find_closest_point(MapArrayXfRow& path_array, StateVector& ego_state){
    
    // get the position of the ego vehicle
    AXf current_ego_pos(2);
    current_ego_pos<<ego_state(0),ego_state(1);
    RCLCPP_INFO(this->get_logger(),"inside find_closest_point ");
    // find the closest pose to the path_array from ego_vehicle
    int R = static_cast<int>(path_array.rows());
    auto xy_block = path_array.block(0, 0, R, 2); // R×2
    Eigen::ArrayXf diffs = (xy_block.rowwise() - current_ego_pos.transpose().row(0)).square().rowwise().sum();
    Eigen::Index idx;
    diffs.minCoeff(&idx);
    RCLCPP_INFO_STREAM(this->get_logger(), "Rows = " <<  diffs.rows() << ", Cols = " << diffs.cols());
    Eigen::Vector2f closest_pt;
    closest_pt << path_array(idx, 0), path_array(idx, 1);
    RCLCPP_INFO_STREAM(this->get_logger(), "closest_pt = " <<  closest_pt(0));
    return idx;
}



// def path_spline(self, x_path, y_path):

//     x_diff = np.diff(x_path)
//     y_diff = np.diff(y_path)

//     phi = np.unwrap(np.arctan2(y_diff, x_diff))
//     phi_init = phi[0]
//     phi = np.hstack(( phi_init, phi  ))

//     arc = np.cumsum( np.sqrt( x_diff**2+y_diff**2 )   )
//     arc_length = arc[-1]

//     arc_vec = np.linspace(0, arc_length, np.shape(x_path)[0])

//     cs_x_path = CubicSpline(arc_vec, x_path)
//     cs_y_path = CubicSpline(arc_vec, y_path)
//     cs_phi_path = CubicSpline(arc_vec, phi)
    
//     return cs_x_path, cs_y_path, cs_phi_path, arc_length, arc_vec



// PathDef ref_wp_spline(Eigen::ArrayXXf ref_wp){



// }



void QpMpc::find_ref_path( StateVector& ego_state){
    Eigen::Index idx = find_closest_point(mat_path_points, ego_state);
    int idx_int  = static_cast<int>(idx);
    int total_rows = static_cast<int>(mat_path_points.rows());
    
    int n = std::min(total_rows,path_num_points);
    
    // Prepare an output array of size (n × 2)
    Eigen::ArrayXXf circular_block(n, 2);
    
    if( n + idx_int <= total_rows){

        circular_block = mat_path_points.block(idx_int, 0,  path_num_points, 2);

    }else{
        // Prepare an output array of size (n × 2)
        int n1 = total_rows - idx_int;
        int n2 = n-n1;
        circular_block.topRows(n1) = mat_path_points.block(idx_int, 0, n1 , 2);
        circular_block.bottomRows(n2) = mat_path_points.block(0, 0, n2 , 2);
    }

    PathDef path_def =   ref_wp_spline(circular_block);
}
