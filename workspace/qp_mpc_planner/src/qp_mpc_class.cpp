#include "qp_mpc_planner/qp_mpc_class.hpp"

QpMpc::QpMpc():Node("qp_mpc_node") {
    RCLCPP_INFO(this->get_logger(),"qp mpc node started");
    std::string map_path = "/root/workspace/src/project_utils/maps/e7_floor5_square.csv";
    load_map(map_path,path_data_points,mat_path_points);
    RCLCPP_INFO(this->get_logger(),"load map successfully called");

}