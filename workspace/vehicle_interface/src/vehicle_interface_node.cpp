#include "rclcpp/rclcpp.hpp"
#include<chrono>
using namespace std::chrono_literals;


class vehicle_class : public rclcpp::Node{




}


int main(int argc, char* argv[]){


    rclcpp::init(argc,argv);
    auto vehicle_node = std::make_shared<vehicle_class>;
    rclcpp::spin(vehicle_node);
    return 0;

}