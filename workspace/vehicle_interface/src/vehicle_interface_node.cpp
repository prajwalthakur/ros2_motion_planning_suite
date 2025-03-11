#include "rclcpp/rclcpp.hpp"
#include "vehicle_interface_class.hpp"
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto vehicle_interface_node = std::make_shared<VehicleInterface>();
    rclcpp::spin(vehicle_interface_node);
    return 0;
}