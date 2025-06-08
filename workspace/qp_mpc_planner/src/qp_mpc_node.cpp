#include <rclcpp/rclcpp.hpp>
#include "qp_mpc_planner/qp_mpc_class.hpp"

int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  auto mpc_class_node = std::make_shared<QpMpc>();
  rclcpp::spin(mpc_class_node);
  return 0;
}